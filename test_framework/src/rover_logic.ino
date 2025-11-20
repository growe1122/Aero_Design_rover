// ===== Rover 4-Motor + I2C Servos — Right-stick arcade =====
// Motors (MD20A drivers):
//   M1(9,8)  = Back Right
//   M2(6,7)  = Front Right
//   M3(3,2)  = Front Left
//   M4(5,4)  = Back Left
//
// Receiver (Spektrum DX6e, Mode 2 or 3):
//   CH1 Aileron  -> D10 (up down)  [PCINT PB2]
//   CH4 Elevator -> D11 (left right)  [PCINT PB3]
//   CH6 Aux1     -> D12 (mode switch: DRIVE<->SERVO) [PCINT PB4]
//   CH5 Aux2     -> D13 (servo snap 90° toggle)      [PCINT PB5]
//
// Servos via PCA9685 (I2C):
//   SERVO_A_CH and SERVO_B_CH mirror right stick in SERVO mode
//   SERVO_SNAP_CH goes to 90° when CH6 high, else neutral
//
// Notes:
// - No physical E-STOP pin. Failsafe (no fresh RX) → motors stop, servos neutral.
// - Per-motor inversion & steer/throttle inversion are configurable below.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ---------------- Pin map ----------------
const uint8_t M1_PWM = 9,  M1_DIR = 8;   // Back right  (RIGHT side)
const uint8_t M2_PWM = 6,  M2_DIR = 7;   // Front right (RIGHT side)
const uint8_t M3_PWM = 3,  M3_DIR = 2;   // Front left  (LEFT side)
const uint8_t M4_PWM = 5,  M4_DIR = 4;   // Back left   (LEFT side)

// RX pins on PORTB (so we can read all with one PCINT vector)
const uint8_t RX_CH1_PIN = 10; // PB2
const uint8_t RX_CH2_PIN = 11; // PB3
const uint8_t RX_CH6_PIN = 12; // PB4  (mode switch)
const uint8_t RX_CH5_PIN = 13; // PB5  (snap switch)

// ---------------- I2C Servo config ----------------
// PCA9685 defaults to 0x40; change if yours is different.
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Pick channels on the PCA9685
const uint8_t SERVO_A_CH    = 0;   // mirrors throttle (U/D) 
const uint8_t SERVO_B_CH    = 3;   // snap servo (CH5) working
const uint8_t SERVO_SNAP_CH = 1;   // L/R brush working

// pulse range for your servos on the PCA9685 (adjust if needed)
const uint16_t SERVO_MIN_US = 1000;
const uint16_t SERVO_MAX_US = 2000;
const uint16_t SERVO_NEUTRAL_US = 1500;
const uint16_t SERVO_SNAP_US    = 1950;  // ~90° (tune for your horn)

// Helper to write microseconds to PCA9685 channel
void pwmWriteUS(uint8_t ch, uint16_t us) {
  // PCA9685 runs at ~50–60 Hz; Adafruit lib maps setPWMFreq properly.
  // Convert microseconds to 12-bit ticks:
  // ticks = us * freq(Hz) * 4096 / 1e6
  // We set freq to 50 Hz below.
  const float freq = 50.0f;
  uint16_t ticks = (uint16_t)constrain((long)(us * freq * 4096.0f / 1000000.0f), 0L, 4095L);
  pwm.setPWM(ch, 0, ticks);
}

// ---------------- Tuning -----------------
const int      PULSE_MIN   = 1000;       // µs
const int      PULSE_CEN   = 1500;
const int      PULSE_MAX   = 2000;
const int      DEAD_US     = 40;
uint8_t        MAX_PWM     = 140;        // soft limit → raise when verified
const uint32_t FAILSAFE_MS = 150;        // stop if no RX update in this time
const int16_t  SLEW        = 8;          // PWM step per loop (ramp)

// Inversion knobs
bool INVERT_STEER     = false;           // flip stick L/R if turn is backwards
bool INVERT_THROTTLE  = false;           // flip stick U/D

// Per-motor flips (if a wheel spins wrong on forward)
bool INV_M1 = false;  // back right
bool INV_M2 = false;  // front right
bool INV_M3 = false;  // front left
bool INV_M4 = false;  // back left

// Mode thresholds (µs) for CH5 with hysteresis
const int MODE_SERVO_THRESHOLD_US = 1400; // below → SERVO mode
const int MODE_DRIVE_THRESHOLD_US = 1600; // above → DRIVE mode

// ---------------- RX state (all on PORTB) ----------------
volatile uint32_t ch1Rise=0, ch2Rise=0, ch5Rise=0, ch6Rise=0;
volatile uint16_t ch1_us=1500, ch2_us=1500, ch5_us=1500, ch6_us=1100;
volatile uint32_t lastRxMicros=0;
volatile uint8_t  prevPortB=0;

// --------------- Helpers ----------------
int16_t usToSigned255(int us) {
  us = constrain(us, PULSE_MIN, PULSE_MAX);
  int d = us - PULSE_CEN;
  if (abs(d) <= DEAD_US) return 0;
  long out = (long)d * 255 / 500;        // map 1000..2000 → -255..+255
  out = constrain(out, -255, 255);
  // gentle expo around center
  const uint8_t EXPO_PCT = 20;           // 0..100
  if (EXPO_PCT > 0) {
    float x = out / 255.0f;
    float expo = x*x*x;
    float mix = (1.0f - EXPO_PCT/100.0f)*x + (EXPO_PCT/100.0f)*expo;
    out = (int16_t)(mix * 255.0f);
  }
  return (int16_t)out;
}

inline void setMotor(uint8_t pwmPin, uint8_t dirPin, int16_t val) {
  int16_t v = constrain(val, -255, 255);
  if (v >= 0) { digitalWrite(dirPin, HIGH); analogWrite(pwmPin, (uint8_t)constrain((int)v, 0, (int)MAX_PWM)); }
  else        { digitalWrite(dirPin, LOW ); analogWrite(pwmPin, (uint8_t)constrain((int)(-v), 0, (int)MAX_PWM)); }
}

inline void stopAll() {
  analogWrite(M1_PWM, 0); analogWrite(M2_PWM, 0);
  analogWrite(M3_PWM, 0); analogWrite(M4_PWM, 0);
}

void driveLR(int16_t left, int16_t right) {
  // Right side = M1 (back-right), M2 (front-right)
  setMotor(M1_PWM, M1_DIR, INV_M1 ? -right : right);
  setMotor(M2_PWM, M2_DIR, INV_M2 ? -right : right);
  // Left side  = M3 (front-left), M4 (back-left)
  setMotor(M3_PWM, M3_DIR, INV_M3 ? -left  : left);
  setMotor(M4_PWM, M4_DIR, INV_M4 ? -left  : left);
}

// --------- Pin-change ISR for D10..D13 (PORTB) ----------
ISR(PCINT0_vect) {
  uint8_t pins = PINB;
  uint8_t changed = pins ^ prevPortB;
  prevPortB = pins;

  uint32_t now = micros();

  // D10 (PB2) CH1 Aileron
  if (changed & _BV(PB2)) {
    if (pins & _BV(PB2)) ch1Rise = now;  // rising
    else if (ch1Rise) {
      ch1_us = (uint16_t)constrain((int32_t)(now - ch1Rise), 900, 2100);
      lastRxMicros = now;
    }
  }
  // D11 (PB3) CH2 Elevator
  if (changed & _BV(PB3)) {
    if (pins & _BV(PB3)) ch2Rise = now;
    else if (ch2Rise) {
      ch2_us = (uint16_t)constrain((int32_t)(now - ch2Rise), 900, 2100);
      lastRxMicros = now;
    }
  }
  // D12 (PB4) CH5 Aux (mode)
  if (changed & _BV(PB4)) {
    if (pins & _BV(PB4)) ch5Rise = now;
    else if (ch5Rise) {
      ch5_us = (uint16_t)constrain((int32_t)(now - ch5Rise), 900, 2100);
      lastRxMicros = now;
    }
  }
  // D13 (PB5) CH6 Aux (snap)
  if (changed & _BV(PB5)) {
    if (pins & _BV(PB5)) ch6Rise = now;
    else if (ch6Rise) {
      ch6_us = (uint16_t)constrain((int32_t)(now - ch6Rise), 900, 2100);
      lastRxMicros = now;
    }
  }
}

// ----------------- Setup/loop -----------------------
void setup() {
  // Motors
  pinMode(M1_DIR, OUTPUT); pinMode(M2_DIR, OUTPUT);
  pinMode(M3_DIR, OUTPUT); pinMode(M4_DIR, OUTPUT);
  pinMode(M1_PWM, OUTPUT); pinMode(M2_PWM, OUTPUT);
  pinMode(M3_PWM, OUTPUT); pinMode(M4_PWM, OUTPUT);
  stopAll();

  // RX pins
  pinMode(RX_CH1_PIN, INPUT);
  pinMode(RX_CH2_PIN, INPUT);
  pinMode(RX_CH5_PIN, INPUT);
  pinMode(RX_CH6_PIN, INPUT);

  // Enable PCINT on D10..D13 (PB2..PB5)
  prevPortB = PINB;                   // snapshot
  PCICR  |= _BV(PCIE0);               // enable PCINT for PORTB
  PCMSK0 |= _BV(PCINT2) | _BV(PCINT3) | _BV(PCINT4) | _BV(PCINT5);

  // I2C Servos
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50.0f); // 50 Hz typical for hobby servos
  pwmWriteUS(SERVO_A_CH,    SERVO_NEUTRAL_US);
  pwmWriteUS(SERVO_B_CH,    SERVO_NEUTRAL_US);
  pwmWriteUS(SERVO_SNAP_CH, SERVO_NEUTRAL_US);

  Serial.begin(115200);
  Serial.println(F("=== Rover: DRIVE<->SERVO on CH5, SNAP on CH6 ==="));
  Serial.print(F("INV_M1=")); Serial.print(INV_M1);
  Serial.print(F(" INV_M2=")); Serial.print(INV_M2);
  Serial.print(F(" INV_M3=")); Serial.print(INV_M3);
  Serial.print(F(" INV_M4=")); Serial.println(INV_M4);
}

void loop() {
  // Copy RX safely
  noInterrupts();
  uint16_t us1 = ch1_us;   // steer
  uint16_t us2 = ch2_us;   // throttle
  uint16_t us5 = ch5_us;   // mode
  uint16_t us6 = ch6_us;   // snap
  uint32_t lastUs = lastRxMicros;
  interrupts();

  static int16_t curL = 0, curR = 0;
  static bool driveMode = true; // start in DRIVE

  // Determine mode with hysteresis
  if (driveMode) {
    if (us5 < MODE_SERVO_THRESHOLD_US) driveMode = false; // → SERVO
  } else {
    if (us5 > MODE_DRIVE_THRESHOLD_US) driveMode = true;  // → DRIVE
  }

  // Failsafe: stale pulses → stop/neutral
  bool stale = (micros() - lastUs) > (FAILSAFE_MS * 1000UL);

  if (driveMode && !stale) {
    // --- DRIVE: right-stick arcade ---
    int16_t steer = usToSigned255(us1);
    int16_t thr   = usToSigned255(us2);
    if (INVERT_STEER)    steer = -steer;
    if (INVERT_THROTTLE) thr   = -thr;

    long targetL = (long)thr + (long)steer;
    long targetR = (long)thr - (long)steer;
    targetL = constrain(targetL, -255, 255);
    targetR = constrain(targetR, -255, 255);

    auto step = [](int16_t cur, int16_t tgt)->int16_t {
      if (tgt > cur) { int16_t n = cur + SLEW; return (n > tgt) ? tgt : n; }
      if (tgt < cur) { int16_t n = cur - SLEW; return (n < tgt) ? tgt : n; }
      return cur;
    };
    curL = step(curL, (int16_t)targetL);
    curR = step(curR, (int16_t)targetR);

    driveLR(curL, curR);

    // hold servos neutral while in DRIVE
    pwmWriteUS(SERVO_A_CH,    SERVO_NEUTRAL_US);
    pwmWriteUS(SERVO_B_CH,    SERVO_NEUTRAL_US);
    pwmWriteUS(SERVO_SNAP_CH, (us6 > 1600) ? SERVO_SNAP_US : SERVO_NEUTRAL_US);
  } else {
    // --- SERVO MODE or STALE: motors idle, servos follow right stick ---
    stopAll();
    curL = 0; curR = 0;

    // map right stick to two servos
    auto usClamp = [](int u){ return (int)constrain(u, SERVO_MIN_US, SERVO_MAX_US); };
    // throttle (U/D) to SERVO_A, steer (L/R) to SERVO_B
    pwmWriteUS(SERVO_A_CH, usClamp(us2));
    pwmWriteUS(SERVO_B_CH, usClamp(us1));
    pwmWriteUS(SERVO_SNAP_CH, (us6 > 1600) ? SERVO_SNAP_US : SERVO_NEUTRAL_US);
  }

  // Telemetry ~20 Hz
  static uint32_t t0 = 0;
  if (millis() - t0 > 50) {
    t0 = millis();
    if (driveMode) {
  Serial.print(F("DRV "));
} else {
  Serial.print(F("SRV "));
}
    Serial.print(F("CH1=")); Serial.print(us1);
    Serial.print(F(" CH2=")); Serial.print(us2);
    Serial.print(F(" CH5=")); Serial.print(us5);
    Serial.print(F(" CH6=")); Serial.println(us6);
  }

  delay(2);
}
