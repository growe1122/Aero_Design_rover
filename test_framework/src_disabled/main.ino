// ===== Rover 4-Motor — Right-stick arcade (CH1@D10 steer, CH2@D11 throttle) =====
// Motors (MD20A drivers):
//   M1(9,8)  = Back Right
//   M2(6,7)  = Front Right
//   M3(3,2)  = Front Left
//   M4(5,4)  = Back Left
// Receiver (DX6e, Mode 2):
//   CH1 Aileron  -> D10 (right stick L/R)
//   CH2 Elevator -> D11 (right stick U/D)
// E-stop: D13 to GND (active LOW)

#include <Arduino.h>

// ---------------- Pin map ----------------
const uint8_t M1_PWM = 9,  M1_DIR = 8;   // Back right
const uint8_t M2_PWM = 6,  M2_DIR = 7;   // Front right
const uint8_t M3_PWM = 3,  M3_DIR = 2;   // Front left
const uint8_t M4_PWM = 5,  M4_DIR = 4;   // Back left

const uint8_t ESTOP_PIN   = 13;          // Active-LOW e-stop (to GND)
const uint8_t RX_CH1_PIN  = 10;          // CH1 Aileron (steer)
const uint8_t RX_CH2_PIN  = 11;          // CH2 Elevator (throttle)

// -------- Per-motor direction invert (flip only wheels that spin backward on straight “forward”) -----
bool INV_M1 = false;   // back right
bool INV_M2 = false;   // front right
bool INV_M3 = false;   // front left
bool INV_M4 = false;   // back left

// -------- Global control inversions --------
bool INV_TURN      = true;   // <— set true if right stick RIGHT currently turns LEFT
bool INVERT_THROTTLE = false; // set true if right stick UP drives backward

// ---------------- Tuning -----------------
const int      PULSE_MIN   = 1000;       // µs
const int      PULSE_CEN   = 1500;       // µs
const int      PULSE_MAX   = 2000;       // µs
const int      DEAD_US     = 40;         // deadband
uint8_t        MAX_PWM     = 140;        // raise after verifying directions
const uint32_t FAILSAFE_MS = 150;        // ms without RX update -> stop
const int16_t  SLEW        = 8;          // PWM step per loop (ramp)

// --------------- RX state ---------------
volatile uint32_t ch1Rise=0, ch2Rise=0;
volatile uint16_t ch1_us=1500, ch2_us=1500;
volatile uint32_t lastRxMicros=0;
volatile uint8_t  prevPortB=0;           // for PCINT on PORTB

// --------------- Helpers ----------------
int16_t usToSigned255(int us) {
  us = constrain(us, PULSE_MIN, PULSE_MAX);
  int d = us - PULSE_CEN;
  if (abs(d) <= DEAD_US) return 0;
  long out = (long)d * 255 / 500;        // map 1000..2000 to -255..+255
  out = constrain(out, -255, 255);
  // mild expo around center
  const uint8_t EXPO_PCT = 20;           // 0..100
  if (EXPO_PCT > 0) {
    float x = out / 255.0f;
    float expo = x*x*x;
    float mix = (1.0f - EXPO_PCT/100.0f)*x + (EXPO_PCT/100.0f)*expo;
    out = (int16_t)(mix * 255.0f);
  }
  return (int16_t)out;
}

inline void setMotor(uint8_t pwm, uint8_t dir, int16_t val) {
  int16_t v = constrain(val, -255, 255);
  if (v >= 0) {
    digitalWrite(dir, HIGH);
    analogWrite(pwm, (uint8_t)constrain((int)v, 0, (int)MAX_PWM));
  } else {
    digitalWrite(dir, LOW);
    analogWrite(pwm, (uint8_t)constrain((int)(-v), 0, (int)MAX_PWM));
  }
}

inline void stopAll() {
  analogWrite(M1_PWM, 0); analogWrite(M2_PWM, 0);
  analogWrite(M3_PWM, 0); analogWrite(M4_PWM, 0);
}

// Right side (M1,M2) takes `right`; Left side (M3,M4) takes `left`
void driveLR(int16_t left, int16_t right) {
  setMotor(M1_PWM, M1_DIR, INV_M1 ? -right : right); // back-right
  setMotor(M2_PWM, M2_DIR, INV_M2 ? -right : right); // front-right
  setMotor(M3_PWM, M3_DIR, INV_M3 ? -left  : left ); // front-left
  setMotor(M4_PWM, M4_DIR, INV_M4 ? -left  : left ); // back-left
}

// --------- Pin-change ISR for D10 & D11 (PORTB) ----------
// PORTB (UNO): D8=PB0, D9=PB1, D10=PB2, D11=PB3, D12=PB4, D13=PB5
ISR(PCINT0_vect) {
  uint8_t pins = PINB;
  uint8_t changed = pins ^ prevPortB;
  prevPortB = pins;

  uint32_t now = micros();

  // D10 (PB2) CH1 Aileron
  if (changed & _BV(PB2)) {
    if (pins & _BV(PB2)) ch1Rise = now;                       // rising
    else if (ch1Rise) {                                       // falling
      ch1_us = (uint16_t)constrain((int32_t)(now - ch1Rise), 900, 2100);
      lastRxMicros = now;
    }
  }

  // D11 (PB3) CH2 Elevator
  if (changed & _BV(PB3)) {
    if (pins & _BV(PB3)) ch2Rise = now;                       // rising
    else if (ch2Rise) {                                       // falling
      ch2_us = (uint16_t)constrain((int32_t)(now - ch2Rise), 900, 2100);
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

  // E-stop
  pinMode(ESTOP_PIN, INPUT_PULLUP);

  // RX pins
  pinMode(RX_CH1_PIN, INPUT);
  pinMode(RX_CH2_PIN, INPUT);

  // Enable PCINT on D10 (PB2) and D11 (PB3)
  prevPortB = PINB;                   // snapshot
  PCICR  |= _BV(PCIE0);               // enable PCINT for PORTB
  PCMSK0 |= _BV(PCINT2) | _BV(PCINT3);// D10, D11

  Serial.begin(115200);
  Serial.println(F("=== Rover Arcade (CH1@D10 steer, CH2@D11 throttle) ==="));
  Serial.println(F("E-stop on D13 (LOW -> stop). Wheels off ground first."));
  Serial.print(F("INV_M1=")); Serial.print(INV_M1);
  Serial.print(F(" INV_M2=")); Serial.print(INV_M2);
  Serial.print(F(" INV_M3=")); Serial.print(INV_M3);
  Serial.print(F(" INV_M4=")); Serial.println(INV_M4);
  Serial.print(F(" INV_TURN=")); Serial.print(INV_TURN);
  Serial.print(F(" INVERT_THROTTLE=")); Serial.println(INVERT_THROTTLE);
}

void loop() {
  // E-stop latch
  if (digitalRead(ESTOP_PIN) == LOW) {
    stopAll();
    while (digitalRead(ESTOP_PIN) == LOW) delay(5);
  }

  // Copy RX safely
  noInterrupts();
  uint16_t ail_us = ch1_us;   // steer (right stick L/R)
  uint16_t ele_us = ch2_us;   // throttle (right stick U/D)
  uint32_t lastUs = lastRxMicros;
  interrupts();

  static int16_t curL = 0, curR = 0;

  // Failsafe if stale pulses
  if ((micros() - lastUs) > (FAILSAFE_MS * 1000UL)) {
    curL = 0; curR = 0;
    driveLR(0, 0);
  } else {
    int16_t steer = usToSigned255(ail_us);
    int16_t thr   = usToSigned255(ele_us);

    if (INV_TURN)        steer = -steer;     // <— flip turn direction here
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

    // Telemetry ~20 Hz
    static uint32_t t0 = 0;
    if (millis() - t0 > 50) {
      t0 = millis();
      Serial.print(F("CH1=")); Serial.print(ail_us);
      Serial.print(F(" CH2=")); Serial.print(ele_us);
      Serial.print(F("  L="));  Serial.print((int)curL);
      Serial.print(F(" R="));   Serial.println((int)curR);
    }
  }

  delay(2);
}
