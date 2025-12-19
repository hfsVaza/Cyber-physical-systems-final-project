import serial
import time
import os

PORT = "COM3"       

GRID_W = 7
GRID_H = 5

MOVE_INTERVAL = 0.15  
USE_CHECKSUM = True    

# Flags 
FLAG_US_VALID = 0x01
FLAG_STILL    = 0x02
FLAG_IMU_OK   = 0x04
FLAG_CALIB    = 0x08
FLAG_H_RISE   = 0x10   # 0 -> >0
FLAG_H_FALL   = 0x20   # >0 -> 0

#  GAME STATE 
player_x, player_y = 0, GRID_H - 1
box_x, box_y = 2, GRID_H - 1
target_x, target_y = 5, 0
carrying = False

last_move_time = 0.0


def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')


def draw_grid(extra_line=""):
    clear_screen()
    for y in range(GRID_H):
        row = []
        for x in range(GRID_W):
            if x == player_x and y == player_y:
                row.append('@' if carrying else 'P')
            elif x == box_x and y == box_y and not carrying:
                row.append('B')
            elif x == target_x and y == target_y:
                row.append('T')
            else:
                row.append('.')
        print(" ".join(row))
    print()
    print("Controls: DX/DY = tilt, H=0 near table, H>0 lifted")
    print("Goal: pick up box B, carry it, drop it on target T")
    if extra_line:
        print(extra_line)


def parse_kv_payload(payload: str):
    parts = payload.split(';')
    data = {}
    for p in parts:
        if '=' in p:
            k, v = p.split('=', 1)
            data[k.strip()] = v.strip()
    return data


def xor_checksum(s: str) -> int:
    c = 0
    for ch in s:
        c ^= ord(ch)
    return c


def verify_and_parse_frame(line: str):

    line = line.strip()
    if not line:
        return False, {}

    if USE_CHECKSUM:
        if ";CS=" not in line:
            return False, {}
        payload, cs_hex = line.rsplit(";CS=", 1)
        try:
            recv_cs = int(cs_hex.strip(), 16)
        except ValueError:
            return False, {}
        calc_cs = xor_checksum(payload)
        if calc_cs != recv_cs:
            return False, {}
        return True, parse_kv_payload(payload)
    else:
        return True, parse_kv_payload(line)


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


def reset_level():
    global player_x, player_y, box_x, box_y, carrying
    player_x, player_y = 0, GRID_H - 1
    box_x, box_y = 2, GRID_H - 1
    carrying = False


def run_calibration(ser: serial.Serial):
    print("=== Calibration ===")
    print("Hold the glove in your comfortable neutral pose and keep it still...")
    ser.write(b"CALIB\n")

    start = time.time()
    timeout_s = 12  # PC side safety timeout

    while True:
        if time.time() - start > timeout_s:
            print("Calibration timeout on PC side (no CALIB_DONE received).")
            break

        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue

        if line.startswith("CALIB_START"):
            print("Calibrating... (collecting samples)")
        elif line.startswith("CALIB_DONE"):
            print("Calibration finished:", line)
            break
        elif line.startswith("CALIB_FAIL"):
            print("Calibration failed:", line)
            break
        else:
            # ignore normal frames during calibration
            pass


def main():
    global player_x, player_y, box_x, box_y, carrying, last_move_time

    print(f"Opening serial port {PORT} @ {BAUD}...")
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)  # wait for Arduino reset

    try:
        ser.write(b"STATUS\n")
        time.sleep(0.1)
        status_line = ser.readline().decode(errors="ignore").strip()
        if status_line.startswith("STATUS"):
            print("Arduino:", status_line)
    except Exception:
        pass

    run_calibration(ser)
    reset_level()
    draw_grid("Ready.")

    # For stability
    last_h = None

    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue

        if line.startswith("CALIB_"):
            continue

        ok, data = verify_and_parse_frame(line)
        if not ok:
            continue

        try:
            dx = int(data.get("DX", "0"))
            dy = int(data.get("DY", "0"))
            h  = int(data.get("H", "-1"))
            flags = int(data.get("F", "0"), 16) if "F" in data else 0
        except ValueError:
            continue

        # Movement 
        now = time.time()
        moved = False
        if now - last_move_time >= MOVE_INTERVAL:
            if dx != 0 or dy != 0:
                # screen Y is inverted: dy=+1 means "up" => y decreases
                player_x = clamp(player_x + dx, 0, GRID_W - 1)
                player_y = clamp(player_y - dy, 0, GRID_H - 1)
                last_move_time = now
                moved = True

        # Determine pick/drop transitions:
        h_rise = (flags & FLAG_H_RISE) != 0
        h_fall = (flags & FLAG_H_FALL) != 0

        if last_h is None:
            last_h = h
        else:
            if not (flags & (FLAG_H_RISE | FLAG_H_FALL)):
                h_rise = (last_h == 0 and h > 0)
                h_fall = (last_h > 0 and h == 0)
            last_h = h

        # PICKUP: only when we detect rising edge from table 
        if not carrying and h_rise:
            if player_x == box_x and player_y == box_y:
                carrying = True
                box_x, box_y = -1, -1  # hide

        # DROP
        if carrying and h_fall:
            if player_x == target_x and player_y == target_y:
                carrying = False
                box_x, box_y = target_x, target_y
                draw_grid(">>> SUCCESS! Box delivered to target.")
                ser.write(b"SUCCESS\n")
                time.sleep(1.0)
                reset_level()
            else:
                carrying = False
                box_x, box_y = player_x, player_y
                draw_grid("XXX FAIL! Box dropped in wrong place.")
                ser.write(b"FAIL\n")
                time.sleep(1.0)
                reset_level()

        # While carrying, keep box hidden
        if carrying:
            box_x, box_y = -1, -1

        # Redraw on move or on interaction 
        if moved or h_rise or h_fall:
            draw_grid(f"DX={dx} DY={dy} H={h} F=0x{flags:02X}")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nExiting.")
