//======================================================================
//  EV TILT ACTUATOR  -  NEMA23 + DM860H  +  BMI160
//  Arduino Mega 2560 (16 MHz)
//
//  MOTOR wiring (Timer3 hardware pulse generator)
//    PUL -> D2   *** must be D2 : it is OC3B ***
//    DIR -> D3
//    ENA -> D4
//
//  BMI160 wiring (I2C)
//    SDA -> D20 , SCL -> D21 , VCC -> 3V3 , GND -> GND , SDO -> GND
//
//  No pin conflict: Timer3 = 2/3/5, I2C = 20/21.
//
//  BEHAVIOUR (one wheel, bench test):
//    tilt > +5 deg  -> STEP_MOVE pulses CW
//    tilt < -5 deg  -> STEP_MOVE pulses CCW
//    within +/-5deg -> hold (deadband)
//
//  ONE_SHOT switch:
//    true  = fire once when tilt crosses 5deg, then wait for it to
//            return inside the band before it can fire again
//    false = keep nudging every loop while tilt stays past 5deg
//======================================================================

#include <Wire.h>

// ------------------------------------------------------------- BMI160
#define BMI160_ADDR    0x68     // 0x69 if SDO tied to VDD
#define REG_CHIP_ID    0x00     // reads 0xD1 when healthy
#define REG_CMD        0x7E
#define REG_ACC_DATA   0x12     // ACC_X_L, 6 bytes
const float ACC_LSB_PER_G = 16384.0f;   // +/- 2 g default range
float lastTilt = 0.0f;

// ------------------------------------------------------------- motor pins
const uint8_t PUL_PIN = 2;      // OC3B - do not move
const uint8_t DIR_PIN = 3;
const uint8_t ENA_PIN = 4;

// ------------------------------------------------------------- mechanics
const uint32_t MICROSTEP = 4000UL;              // DM860H DIP setting
const uint32_t GEAR      = 15UL;                // 1:15 gearbox
const uint32_t STEPS_REV = MICROSTEP * GEAR;    // 60 000 pulses / OUTPUT rev

// ------------------------------------------------------------- motion profile
const uint32_t V_START =   1000UL;   // steps/s  start/stop rate
const uint32_t V_MAX   =  60000UL;   // steps/s  cruise (your tuned value)
const uint32_t ACCEL   = 400000UL;   // steps/s^2

// ------------------------------------------------------------- tilt control
const float    DEADBAND_DEG = 5.0f;  // no motion inside +/- this
const uint32_t STEP_MOVE    = 2000UL;// pulses per actuation
const bool     ONE_SHOT     = true;  // fire once per crossing

// ------------------------------------------------------------- constants
const uint32_t TIMER_HZ = 2000000UL; // 16 MHz / 8
const uint32_t V_LIMIT  =  200000UL;

volatile uint32_t stepsLeft = 0;
volatile bool     running   = false;
bool armed = true;                    // used only when ONE_SHOT is true

//======================================================================
//  MOTOR
//======================================================================
void setRate(uint32_t hz)
{
  if (hz < 31)      hz = 31;
  if (hz > V_LIMIT) hz = V_LIMIT;
  uint16_t top  = (uint16_t)(TIMER_HZ / hz) - 1;
  uint16_t high = (top + 1) >> 1;
  if (high < 5) high = 5;
  OCR3A = top;
  OCR3B = high;
}

void startMove(uint32_t steps, bool cw)
{
  digitalWrite(DIR_PIN, cw ? HIGH : LOW);
  delayMicroseconds(50);
  stepsLeft = steps;
  running   = true;
  setRate(V_START);
  TCNT3  = 0;
  TIFR3  = _BV(OCF3B);
  TIMSK3 = _BV(OCIE3B);
  TCCR3A = _BV(COM3B1) | _BV(WGM31) | _BV(WGM30);
  TCCR3B = _BV(WGM33)  | _BV(WGM32) | _BV(CS31);
}

ISR(TIMER3_COMPB_vect)
{
  if (--stepsLeft == 0) {
    TCCR3A = 0;
    TCCR3B = 0;
    TIMSK3 = 0;
    running = false;
  }
}

void move(uint32_t steps, bool cw)
{
  if (steps < 4) return;
  uint32_t ramp = (uint32_t)(((float)V_MAX * V_MAX - (float)V_START * V_START)
                             / (2.0f * ACCEL));
  if (ramp * 2 > steps) ramp = steps / 2;
  startMove(steps, cw);
  for (;;) {
    noInterrupts();
    uint32_t left = stepsLeft;
    bool     go   = running;
    interrupts();
    if (!go) break;
    uint32_t done = steps - left;
    uint32_t v;
    if      (done < ramp) v = (uint32_t)sqrt((float)V_START * V_START + 2.0f * ACCEL * done);
    else if (left < ramp) v = (uint32_t)sqrt((float)V_START * V_START + 2.0f * ACCEL * left);
    else                  v = V_MAX;
    if (v > V_MAX)   v = V_MAX;
    if (v < V_START) v = V_START;
    setRate(v);
  }
}

//======================================================================
//  BMI160
//======================================================================
void writeReg(uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

float readTilt()
{
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(REG_ACC_DATA);
  Wire.endTransmission(false);
  Wire.requestFrom(BMI160_ADDR, 6);
  if (Wire.available() < 6) return lastTilt;   // read failed, reuse last

  // read low-then-high explicitly (order matters - never one-line it)
  uint8_t xl = Wire.read(), xh = Wire.read();
  uint8_t yl = Wire.read(), yh = Wire.read();
  uint8_t zl = Wire.read(), zh = Wire.read();
  int16_t ax = (int16_t)(xl | (xh << 8));
  int16_t ay = (int16_t)(yl | (yh << 8));
  int16_t az = (int16_t)(zl | (zh << 8));

  // Swap ax/ay here if the wrong tilt axis responds on your mount.
  float tilt = atan2((float)ax, (float)az) * 57.29578f;   // rad -> deg
  lastTilt = tilt;
  return tilt;
}

//======================================================================
//  SETUP / LOOP
//======================================================================
void setup()
{
  Serial.begin(9600);
  Wire.begin();                 // Mega: SDA=20, SCL=21 automatically

  pinMode(PUL_PIN, OUTPUT);  digitalWrite(PUL_PIN, LOW);
  pinMode(DIR_PIN, OUTPUT);  digitalWrite(DIR_PIN, LOW);
  pinMode(ENA_PIN, OUTPUT);  digitalWrite(ENA_PIN, LOW);
  TCCR3A = 0; TCCR3B = 0; TIMSK3 = 0;

  // confirm the sensor is alive before trusting any angle
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(REG_CHIP_ID);
  Wire.endTransmission(false);
  Wire.requestFrom(BMI160_ADDR, 1);
  uint8_t id = Wire.available() ? Wire.read() : 0x00;
  Serial.print(F("CHIP_ID = 0x"));
  Serial.print(id, HEX);
  Serial.println(id == 0xD1 ? F("  (OK)") : F("  (WRONG - check wiring)"));

  writeReg(REG_CMD, 0x11);      // accel -> normal mode
  delay(300);                   // sensor + driver settle
  Serial.println(F("ready\n"));
}

void loop()
{
  float tilt = readTilt();
  Serial.print(F("tilt = "));
  Serial.print(tilt, 1);
  Serial.print(F(" deg   "));

  bool outside = (tilt > DEADBAND_DEG) || (tilt < -DEADBAND_DEG);

  if (ONE_SHOT) {
    if (outside && armed) {
      if (tilt > 0) { Serial.println(F("-> CW  (one-shot)")); move(STEPS_REV, true);  }
      else          { Serial.println(F("-> CCW (one-shot)")); move(STEPS_REV, false); }
      armed = false;                     // block until tilt returns
    }
    else if (!outside) {
      armed = true;                      // re-arm inside the band
      Serial.println(F("-> hold (armed)"));
    }
    else {
      Serial.println(F("-> hold (spent)"));
    }
  }
  else {   // repeat mode: nudge every loop while tilted
    if      (tilt > DEADBAND_DEG)  { Serial.println(F("-> CW"));  move(STEPS_REV, true);  }
    else if (tilt < -DEADBAND_DEG) { Serial.println(F("-> CCW")); move(STEPS_REV, false); }
    else                            Serial.println(F("-> hold"));
  }

  delay(50);
}