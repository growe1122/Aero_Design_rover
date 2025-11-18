// ===== Rover 4-Motor — Right-Stick Arcade Drive (DX6e) =====
// Right stick ONLY:
//   - Right/Left  -> Steering  (Aileron = CH1 @ D10)
//   - Up/Down     -> Throttle  (Elevator = CH2 @ D11)
// Motors: M1(9,8) M2(6,7) M3(3,2) M4(5,4) ; E-STOP D13 (LOW)

#include <Arduino.h>

// ---------------- Pin map ----------------
const uint8_t M1_PWM = 9,  M1_DIR = 8;
const uint8_t M2_PWM = 6,  M2_DIR = 7;
const uint8_t M3_PWM = 3,  M3_DIR = 2;
const uint8_t M4_PWM = 5,  M4_DIR = 4;
const uint8_t ESTOP_PIN  = 13;

const uint8_t RX_AILERON_PIN  = 10;   // CH1 (Right stick L/R)
const uint8_t RX_ELEVATOR_PIN = 11;   // CH2 (Right stick U/D)

// ---------------- Tuning -----------------
const int      PULSE_MIN   = 1000, PULSE_CEN = 1500, PULSE_MAX = 2000;
const int      DEAD_US     = 40;
const uint8_t  EXPO_PCT    = 20;      // 0..100 (mild smoothing near center)
uint8_t        MAX_PWM     = 120;     // raise after confirming directions
const uint32_t FAILSAFE_MS = 120;
const int16_t  SLEW        = 8;       // PWM step per loop (~2ms)

// Optional flips if directions feel wrong (set to true to invert)
bool INVERT_STEER   = false;          // flip CH1
bool INVERT_THROTTLE= false;          // flip CH2

// --------------- RX state ---------------
volatile uint32_t chAilRise=0, chEleRise=0;
volatile uint16_t chAil_us=1500, chEle_us=1500;
volatile uint32_t lastRxMicros=0;
volatile uint8_t  prevPortB=0;

// --------------- Helpers ----------------
int16_t usToSigned255(int us) {
  us = constrain(us, PULSE_MIN, PULSE_MAX);
  int d = us - PULSE_CEN;
  if (abs(d) <= DEAD_US) return 0;
  long out = (long)d * 255 / 500;       // 1000→-255, 1500→0, 2000→+255
  out = constrain(out, -255, 255);
  if (EXPO_PCT > 0) {
    float x = out / 255.0f; float expo = x*x*x;
    float mix = (1.0f - EXPO_PCT/100.0f)*x + (EXPO_PCT/100.0f)*expo;
    out = (int16_t)(mix * 255.0f);
  }
  return (int16_t)out;
}

inline void setMotor(uint8_t pwm, uint8_t dir, int16_t val) {
  int16_t v = constrain(val, -255, 255);
  if (v >= 0) { digitalWrite(dir, HIGH);  analogWrite(pwm, constrain((int)v, 0, (int)MAX_PWM)); }
  else        { digitalWrite(dir, LOW);   analogWrite(pwm, constrain((int)(-v), 0, (int)MAX_PWM)); }
}

inline void stopAll() {
  analogWrite(M1_PWM,0); analogWrite(M2_PWM,0);
  analogWrite(M3_PWM,0); analogWrite(M4_PWM,0);
}

void driveLR(int16_t left, int16_t right) {
  setMotor(M1_PWM, M1_DIR, left);
  setMotor(M3_PWM, M3_DIR, left);
  setMotor(M2_PWM, M2_DIR, right);
  setMotor(M4_PWM, M4_DIR, right);
}

// --------- Pin-change ISR for D10/D11 (PORTB) ----------
ISR(PCINT0_vect) {
  uint8_t pins = PINB, changed = pins ^ prevPortB; prevPortB = pins;
  uint32_t now = micros();

  // D10 (PB2) Aileron
  if (changed & _BV(PB2)) {
    if (pins & _BV(PB2)) chAilRise = now;
    else if (chAilRise) { chAil_us = (uint16_t)constrain((int32_t)(now - chAilRise), 900, 2100); lastRxMicros = now; }
  }
  // D11 (PB3) Elevator
  if (changed & _BV(PB3)) {
    if (pins & _BV(PB3)) chEleRise = now;
    else if (chEleRise) { chEle_us = (uint16_t)constrain((int32_t)(now - chEleRise), 900, 2100); lastRxMicros = now; }
  }
}

// ----------------- Setup/loop -----------------------
void setup() {
  pinMode(M1_DIR,OUTPUT); pinMode(M2_DIR,OUTPUT);
  pinMode(M3_DIR,OUTPUT); pinMode(M4_DIR,OUTPUT);
  pinMode(M1_PWM,OUTPUT); pinMode(M2_PWM,OUTPUT);
  pinMode(M3_PWM,OUTPUT); pinMode(M4_PWM,OUTPUT);
  stopAll();

  pinMode(ESTOP_PIN, INPUT_PULLUP);
  pinMode(RX_AILERON_PIN, INPUT);
  pinMode(RX_ELEVATOR_PIN, INPUT);

  prevPortB = PINB;
  PCICR  |= _BV(PCIE0);                      // enable PCINT for PORTB
  PCMSK0 |= _BV(PCINT2) | _BV(PCINT3);       // D10, D11

  Serial.begin(115200);
  Serial.println(F("=== Right-Stick Arcade Drive (CH1 steer, CH2 throttle) ==="));
}

void loop() {
  if (digitalRead(ESTOP_PIN)==LOW) { stopAll(); while(digitalRead(ESTOP_PIN)==LOW) delay(5); }

  noInterrupts();
  uint16_t ail = chAil_us;
  uint16_t ele = chEle_us;
  uint32_t lastUs = lastRxMicros;
  interrupts();

  static int16_t curL=0, curR=0;

  if ((micros() - lastUs) > (FAILSAFE_MS*1000UL)) {
    curL=0; curR=0; driveLR(0,0);
  } else {
    int16_t steer = usToSigned255(ail);
    int16_t thr   = usToSigned255(ele);
    if (INVERT_STEER)    steer = -steer;
    if (INVERT_THROTTLE) thr   = -thr;

    // Right-stick arcade mixing
    long targetL = (long)thr + (long)steer;
    long targetR = (long)thr - (long)steer;
    targetL = constrain(targetL, -255, 255);
    targetR = constrain(targetR, -255, 255);

    // Slew limiting for smoothness
    auto step = [](int16_t cur, int16_t tgt)->int16_t{
      if (tgt > cur)  return (int16_t)min<int>(cur + SLEW, tgt);
      if (tgt < cur)  return (int16_t)max<int>(cur - SLEW, tgt);
      return cur;
    };
    curL = step(curL, (int16_t)targetL);
    curR = step(curR, (int16_t)targetR);

    driveLR(curL, curR);

    static uint32_t t0=0;
    if (millis()-t0 > 50) {
      t0 = millis();
      Serial.print(F("CH1=")); Serial.print(ail);
      Serial.print(F(" CH2=")); Serial.print(ele);
      Serial.print(F("  L="));  Serial.print((int)curL);
      Serial.print(F(" R="));   Serial.print((int)curR);
      Serial.print(F("  MAX_PWM=")); Serial.println(MAX_PWM);
    }
  }
  delay(2);
}
