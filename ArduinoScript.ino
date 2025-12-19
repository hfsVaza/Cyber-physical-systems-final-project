#include <Wire.h>
#include <math.h>

// AVR bare-metal headers for sleep,interrupts + power reduction
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>

// HC-SR04
const uint8_t TRIG_PIN   = 8;   // D8 = PB0
const uint8_t ECHO_PIN   = 9;   // D9 = PB1

// LEDs (actuators)
const uint8_t LED_GREEN  = 3;   // D3 = PD3
const uint8_t LED_RED    = 4;   // D4 = PD4

const uint8_t NODE_ID = 1;

// MPU6886 I2C address 
uint8_t imu_addr = 0x68;

// Axis mapping 
const bool SWAP_XY = false;     // if true swap DX/DY
const int8_t DX_SIGN = +1;      // invert if right tilt moves left
const int8_t DY_SIGN = +1;      // invert if up tilt moves down

// Filtering responsiveness 
const uint8_t ALPHA_NUM = 3;    
const uint8_t ALPHA_DEN = 10;

// Ultrasonic height thresholds 
const int16_t H_NEAR_CM = 6;    // <6 cm - H=0 
const int16_t H_MID_CM  = 15;   // <15 cm - H=1, else H=2

volatile bool sample_now = false;   // set by Timer1 ISR
bool sleepEnabled = true;           // SLEEPON/SLEEPOFF

bool imu_ok = false;
bool calibrated = false;

// pitch/roll in centi-degrees 
int16_t pitch_offset_cd = 0;
int16_t roll_offset_cd  = 0;

int16_t pitch_filt_cd = 0;
int16_t roll_filt_cd  = 0;
int16_t prev_pitch_cd = 0;
int16_t prev_roll_cd  = 0;

// Dynamic thresholds set by calibration 
int16_t deadzone_cd = 500;        
int16_t strong_cd   = 1500;       
int16_t still_th_cd = 150;        

// Height tracking for transition flags
int8_t last_h = -1;

// Command parser buffer
char cmdBuf[32];
uint8_t cmdLen = 0;

// Feedback 
enum FeedbackMode { FB_NONE, FB_SUCCESS, FB_FAIL };
FeedbackMode fbMode = FB_NONE;
uint8_t fbTogglesLeft = 0;
unsigned long fbNextToggleMs = 0;
bool fbLedOn = false;

void setupTimer1_20Hz() {
  // Tick = 16MHz/1024 = 15625 Hz. For 20 Hz = 15625/20 = 781 counts
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 780;  // 781 ticks 

  TCCR1B |= (1 << WGM12);                 // CTC mode
  TCCR1B |= (1 << CS12) | (1 << CS10);    // prescaler 1024
  TIMSK1 |= (1 << OCIE1A);                // compare A interrupt enable
  sei();
}

ISR(TIMER1_COMPA_vect) {
  sample_now = true;
}

// Utility: XOR checksum
uint8_t xorChecksum(const char* s) {
  uint8_t c = 0;
  while (*s) c ^= (uint8_t)(*s++);
  return c;
}

// IMU (MPU6886-style) 
bool imuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(imu_addr);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

bool imuReadBytes(uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(imu_addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;

  uint8_t got = Wire.requestFrom(imu_addr, n);
  if (got != n) return false;

  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

bool imuBeginAt(uint8_t addr) {
  imu_addr = addr;

  // Wake up device: PWR_MGMT_1 (0x6B) = 0x00
  if (!imuWrite(0x6B, 0x00)) return false;
  delay(10);

  uint8_t who = 0;
  if (!imuReadBytes(0x75, &who, 1)) return false;

  if (who == 0xFF || who == 0x00) return false;

  return true;
}

bool imuBeginAuto() {
  if (imuBeginAt(0x68)) return true;
  if (imuBeginAt(0x69)) return true;
  return false;
}

bool readPitchRollCentiDeg(int16_t &pitch_cd, int16_t &roll_cd) {
  // ACCEL_XOUT_H.ZOUT_L starting at 0x3B 
  uint8_t b[6];
  if (!imuReadBytes(0x3B, b, 6)) return false;

  int16_t ax = (int16_t)((b[0] << 8) | b[1]);
  int16_t ay = (int16_t)((b[2] << 8) | b[3]);
  int16_t az = (int16_t)((b[4] << 8) | b[5]);

  float fax = (float)ax;
  float fay = (float)ay;
  float faz = (float)az;

  float roll  = atan2f(fay, faz) * 57.29578f;
  float pitch = atan2f(-fax, sqrtf(fay*fay + faz*faz)) * 57.29578f;

  pitch_cd = (int16_t)(pitch * 100.0f);
  roll_cd  = (int16_t)(roll  * 100.0f);
  return true;
}

//Ultrasonic 
int16_t readUltrasonicCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // timeout 25000us ~ 4.3m range
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 25000);
  if (duration == 0) return -1;
  return (int16_t)(duration / 58UL);
}

int8_t heightLevelFromCm(int16_t cm) {
  if (cm < 0) return -1;
  if (cm < H_NEAR_CM) return 0;
  if (cm < H_MID_CM)  return 1;
  return 2;
}

//Directions (DX/DY) 
void computeDirections(int16_t pitch_cd, int16_t roll_cd, int8_t &dx, int8_t &dy) {
  const float DOM_RATIO = 1.2f;

  float ap = fabsf((float)pitch_cd);
  float ar = fabsf((float)roll_cd);

  dx = 0; dy = 0;

  if (ap > ar * DOM_RATIO) {
    if (pitch_cd > strong_cd)        dy = 1;
    else if (pitch_cd < -strong_cd)  dy = -1;
    else if (abs(pitch_cd) < deadzone_cd) dy = 0;
    else dy = (pitch_cd > 0) ? 1 : -1;
  } else if (ar > ap * DOM_RATIO) {
    if (roll_cd > strong_cd)         dx = 1;
    else if (roll_cd < -strong_cd)   dx = -1;
    else if (abs(roll_cd) < deadzone_cd) dx = 0;
    else dx = (roll_cd > 0) ? 1 : -1;
  } else {
    dx = 0; dy = 0;
  }

  int8_t out_dx = dx;
  int8_t out_dy = dy;

  if (SWAP_XY) {
    int8_t tmp = out_dx;
    out_dx = out_dy;
    out_dy = tmp;
  }

  dx = out_dx * DX_SIGN;
  dy = out_dy * DY_SIGN;
}

// Calibration 
bool calibrating = false;

const uint16_t CALIB_SAMPLES = 120; // 20Hz -> ~6 seconds

const unsigned long CALIB_TIMEOUT_MS = 8000;  // hard stop
const uint16_t MIN_CALIB_SAMPLES = 30;        

unsigned long calib_start_ms = 0;
uint16_t calib_count = 0;

// Running sums for mean/std 
int32_t sum_p = 0, sum_r = 0;
int64_t sumsq_p = 0, sumsq_r = 0;

void failCalibration(const char* reason) {
  calibrating = false;
  calibrated = false;

  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, LOW);

  Serial.print("CALIB_FAIL;REASON=");
  Serial.print(reason);
  Serial.print(";N=");
  Serial.print(calib_count);
  Serial.print(";IMU=");
  Serial.println(imu_ok ? "OK" : "NO");
}

void startCalibration() {
  calibrating = true;
  calib_start_ms = millis();

  calib_count = 0;
  sum_p = sum_r = 0;
  sumsq_p = sumsq_r = 0;

  // Reset filters 
  pitch_filt_cd = 0;
  roll_filt_cd  = 0;
  prev_pitch_cd = 0;
  prev_roll_cd  = 0;

  Serial.println("CALIB_START");
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_RED, HIGH);
}

void finishCalibration() {
  if (calib_count == 0) return;

  int16_t mean_p = (int16_t)(sum_p / (int32_t)calib_count);
  int16_t mean_r = (int16_t)(sum_r / (int32_t)calib_count);

  int32_t ex2_p = (int32_t)(sumsq_p / (int64_t)calib_count);
  int32_t ex2_r = (int32_t)(sumsq_r / (int64_t)calib_count);

  int32_t var_p = ex2_p - (int32_t)mean_p * (int32_t)mean_p;
  int32_t var_r = ex2_r - (int32_t)mean_r * (int32_t)mean_r;

  if (var_p < 0) var_p = 0;
  if (var_r < 0) var_r = 0;

  int16_t std_p = (int16_t)(sqrtf((float)var_p));
  int16_t std_r = (int16_t)(sqrtf((float)var_r));

  pitch_offset_cd = mean_p;
  roll_offset_cd  = mean_r;

  int16_t noise_cd = (std_p > std_r) ? std_p : std_r;
  if (noise_cd < 50) noise_cd = 50;

  deadzone_cd = noise_cd * 3;
  if (deadzone_cd < 500) deadzone_cd = 500;

  strong_cd = deadzone_cd * 3;
  if (strong_cd < 1500) strong_cd = 1500;

  still_th_cd = noise_cd * 3;
  if (still_th_cd < 150) still_th_cd = 150;

  calibrated = true;
  calibrating = false;

  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, LOW);

  Serial.print("CALIB_DONE;PO=");
  Serial.print(pitch_offset_cd);
  Serial.print(";RO=");
  Serial.print(roll_offset_cd);
  Serial.print(";DZ=");
  Serial.print(deadzone_cd);
  Serial.print(";ST=");
  Serial.println(still_th_cd);
}

// Non blocking feedback patterns 
void startFeedback(FeedbackMode mode) {
  fbMode = mode;
  fbTogglesLeft = 6;
  fbNextToggleMs = millis();
  fbLedOn = false;
}

void updateFeedback(unsigned long nowMs) {
  if (fbMode == FB_NONE) return;
  if (fbTogglesLeft == 0) {
    fbMode = FB_NONE;
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, LOW);
    return;
  }
  if (nowMs >= fbNextToggleMs) {
    fbLedOn = !fbLedOn;
    if (fbMode == FB_SUCCESS) {
      digitalWrite(LED_GREEN, fbLedOn ? HIGH : LOW);
      digitalWrite(LED_RED, LOW);
    } else if (fbMode == FB_FAIL) {
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_RED, fbLedOn ? HIGH : LOW);
    }
    fbTogglesLeft--;
    fbNextToggleMs = nowMs + 150;
  }
}

// Serial command handling (line-based) 
void printStatus() {
  Serial.print("STATUS;IMU=");
  Serial.print(imu_ok ? "OK" : "NO");
  Serial.print(";ADDR=0x");
  Serial.print(imu_addr, HEX);
  Serial.print(";CAL=");
  Serial.print(calibrated ? "YES" : "NO");
  Serial.print(";DZ=");
  Serial.print(deadzone_cd);
  Serial.print(";ST=");
  Serial.println(still_th_cd);
}

void processCommand(const char* cmd) {
  if (strcmp(cmd, "SUCCESS") == 0) {
    startFeedback(FB_SUCCESS);
  } else if (strcmp(cmd, "FAIL") == 0) {
    startFeedback(FB_FAIL);
  } else if (strcmp(cmd, "IDLE") == 0) {
    fbMode = FB_NONE;
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, LOW);
  } else if (strcmp(cmd, "CALIB") == 0) {
    if (!imu_ok) {
      failCalibration("NO_IMU");
    } else {
      startCalibration();
    }
  } else if (strcmp(cmd, "SLEEPON") == 0) {
    sleepEnabled = true;
  } else if (strcmp(cmd, "SLEEPOFF") == 0) {
    sleepEnabled = false;
  } else if (strcmp(cmd, "STATUS") == 0) {
    printStatus();
  }
}

void pollSerialCommands() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      cmdBuf[cmdLen] = '\0';
      if (cmdLen > 0) processCommand(cmdBuf);
      cmdLen = 0;
    } else {
      if (cmdLen < sizeof(cmdBuf) - 1) {
        cmdBuf[cmdLen++] = c;
      }
    }
  }
}

// Setup and Loop 
void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, LOW);

  Wire.begin();

  imu_ok = imuBeginAuto();

  // disable unused peripherals 
  power_adc_disable();
  power_spi_disable();

  setupTimer1_20Hz();
  last_h = -1;

  printStatus();
}

void loop() {
  unsigned long nowMs = millis();

  pollSerialCommands();
  updateFeedback(nowMs);

  if (sample_now) {
    sample_now = false;

    int16_t dist_cm = readUltrasonicCm();
    int8_t  h = heightLevelFromCm(dist_cm);

    int16_t pitch_raw = 0, roll_raw = 0;
    bool imu_read_ok = false;
    if (imu_ok) {
      imu_read_ok = readPitchRollCentiDeg(pitch_raw, roll_raw);
    }

    // Calibration mode 
    if (calibrating) {
      if (imu_read_ok) {
        sum_p += pitch_raw;
        sum_r += roll_raw;
        sumsq_p += (int32_t)pitch_raw * (int32_t)pitch_raw;
        sumsq_r += (int32_t)roll_raw  * (int32_t)roll_raw;
        calib_count++;
      }

      if (calib_count >= CALIB_SAMPLES) {
        finishCalibration();
      } else if (nowMs - calib_start_ms >= CALIB_TIMEOUT_MS) {
        if (calib_count >= MIN_CALIB_SAMPLES) finishCalibration();
        else failCalibration("TIMEOUT");
      }
      return; 
    }

    // offsets
    int16_t pitch_cd = pitch_raw - (calibrated ? pitch_offset_cd : 0);
    int16_t roll_cd  = roll_raw  - (calibrated ? roll_offset_cd  : 0);

    // Filter
    pitch_filt_cd = (int16_t)((ALPHA_NUM * (int32_t)pitch_cd + (ALPHA_DEN - ALPHA_NUM) * (int32_t)pitch_filt_cd) / ALPHA_DEN);
    roll_filt_cd  = (int16_t)((ALPHA_NUM * (int32_t)roll_cd  + (ALPHA_DEN - ALPHA_NUM) * (int32_t)roll_filt_cd ) / ALPHA_DEN);

    // Neutral detection
    int16_t dP = pitch_filt_cd - prev_pitch_cd;
    int16_t dR = roll_filt_cd  - prev_roll_cd;
    prev_pitch_cd = pitch_filt_cd;
    prev_roll_cd  = roll_filt_cd;

    bool hand_still = (abs(dP) < still_th_cd) && (abs(dR) < still_th_cd);

    // Directions
    int8_t dx = 0, dy = 0;
    computeDirections(pitch_filt_cd, roll_filt_cd, dx, dy);

    // Height transition 
    bool h_rise = false; 
    bool h_fall = false; 
    if (last_h != -1 && h != -1) {
      if (last_h == 0 && h > 0) h_rise = true;
      if (last_h > 0 && h == 0) h_fall = true;
    }
    last_h = h;

    // Flags
    uint8_t flags = 0;
    if (dist_cm >= 0)        flags |= 0x01; // ultrasonic valid
    if (hand_still)          flags |= 0x02; // hand still
    if (imu_ok && imu_read_ok) flags |= 0x04; // imu fine
    if (calibrated)          flags |= 0x08; // calibrated
    if (h_rise)              flags |= 0x10; // height rise
    if (h_fall)              flags |= 0x20; // height fall

// payload
    char payload[128];
    snprintf(payload, sizeof(payload),
             "ID=%u;T=%lu;DX=%d;DY=%d;H=%d;P=%d;R=%d;F=%02X",
             NODE_ID, nowMs, dx, dy, h, pitch_filt_cd, roll_filt_cd, flags);

    uint8_t cs = xorChecksum(payload);

    char line[160];
    snprintf(line, sizeof(line), "%s;CS=%02X", payload, cs);
    Serial.println(line);
  }

  // Sleep between samples 
  if (sleepEnabled) {
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();

    noInterrupts();
    bool canSleep = !sample_now && !Serial.available();
    interrupts();

    if (canSleep) sleep_cpu();
    sleep_disable();
  }
}
