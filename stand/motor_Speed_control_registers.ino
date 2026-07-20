//======================================================================
//  NEMA23 + DM860H  -  hardware-timer step generator
//  Arduino Mega 2560 (16 MHz)
//
//  Wiring
//    PUL  ->  D2      *** must be D2 : it is OC3B ***
//    DIR  ->  D3
//    ENA  ->  D4
//
//  Timer3 runs in Fast PWM mode 15 (TOP = OCR3A), prescaler /8:
//      1 tick      = 0.5 us
//      step rate   = 2 000 000 / (OCR3A + 1)   Hz
//      pulse high  = OCR3B ticks
//
//  OCR3A and OCR3B are double buffered in this mode, so the frequency
//  can be changed mid-move and the hardware always finishes the pulse
//  it started - no runt pulses, no jitter, no missed steps.
//
//  The CPU never touches the pin. It only re-plans the speed.
//======================================================================

// ------------------------------------------------------------- pins
const uint8_t PUL_PIN = 2;      // OC3B - do not move this one
const uint8_t DIR_PIN = 3;
const uint8_t ENA_PIN = 4;

// ------------------------------------------------------------- mechanics
const uint32_t MICROSTEP = 2000UL;              // DM860H DIP setting
const uint32_t GEAR      = 15UL;                // 1:20 gearbox
const uint32_t STEPS_REV = MICROSTEP * GEAR;    // 80 000 pulses per OUTPUT rev

// ------------------------------------------------------------- profile (TUNE THESE)
const uint32_t V_START =   1000UL;   // steps/s    start/stop rate, keep low
const uint32_t V_MAX   =  90000UL;   // steps/s -> 900 rpm motor, 45 rpm output
const uint32_t ACCEL   = 400000UL;   // steps/s^2

// ------------------------------------------------------------- constants
const uint32_t TIMER_HZ = 2000000UL; // 16 MHz / 8
const uint32_t V_LIMIT  =  200000UL; // DM860H max pulse input - check your manual

volatile uint32_t stepsLeft = 0;
volatile bool     running   = false;

// ---------------------------------------------------------------------
// Program the pulse frequency. Safe to call at any time while running.
void setRate(uint32_t hz)
{
  if (hz < 31)      hz = 31;          // below this OCR3A overflows 16 bits
  if (hz > V_LIMIT) hz = V_LIMIT;

  uint16_t top  = (uint16_t)(TIMER_HZ / hz) - 1;
  uint16_t high = (top + 1) >> 1;     // 50 % duty
  if (high < 5) high = 5;             // >= 2.5 us high  (DM860H minimum)

  OCR3A = top;                        // both applied at the next BOTTOM
  OCR3B = high;
}

// ---------------------------------------------------------------------
void startMove(uint32_t steps, bool cw)
{
  digitalWrite(DIR_PIN, cw ? HIGH : LOW);
  delayMicroseconds(50);              // DIR setup time before first pulse

  stepsLeft = steps;
  running   = true;
  setRate(V_START);

  TCNT3  = 0;
  TIFR3  = _BV(OCF3B);                                  // clear stale flag
  TIMSK3 = _BV(OCIE3B);                                 // int on falling edge
  TCCR3A = _BV(COM3B1) | _BV(WGM31) | _BV(WGM30);       // OC3B non-inverting
  TCCR3B = _BV(WGM33)  | _BV(WGM32) | _BV(CS31);        // mode 15, /8  -> GO
}

// Fires once per pulse, on the falling edge. Keep it tiny.
ISR(TIMER3_COMPB_vect)
{
  if (--stepsLeft == 0) {
    TCCR3A = 0;          // release OC3B; pin reverts to PORTE4, which is LOW
    TCCR3B = 0;          // stop the clock
    TIMSK3 = 0;
    running = false;
  }
}

// ---------------------------------------------------------------------
// Blocking trapezoidal move.
// Speed is derived from POSITION, not elapsed time, so it cannot drift:
//        v = sqrt(V_START^2 + 2 * ACCEL * travelled)
void move(uint32_t steps, bool cw)
{
  if (steps < 4) return;

  // steps needed to wind up from V_START to V_MAX
  uint32_t ramp = (uint32_t)(((float)V_MAX * V_MAX - (float)V_START * V_START)
                             / (2.0f * ACCEL));
  if (ramp * 2 > steps) ramp = steps / 2;    // short move -> triangular profile

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

// ---------------------------------------------------------------------
void setup()
{
  pinMode(PUL_PIN, OUTPUT);  digitalWrite(PUL_PIN, LOW);
  pinMode(DIR_PIN, OUTPUT);  digitalWrite(DIR_PIN, LOW);
  pinMode(ENA_PIN, OUTPUT);  digitalWrite(ENA_PIN, LOW);   // see note below

  TCCR3A = 0; TCCR3B = 0; TIMSK3 = 0;   // Timer3 belongs to us now
  delay(300);                           // let the driver come up
}

void loop()
{
  move(STEPS_REV, true);     // one full OUTPUT-shaft revolution, CW
  delay(500);
  move(STEPS_REV, true);    // and back
  delay(500);
}

//======================================================================
//  NOTES
//
//  * Timer3 owns pins 2, 3 and 5, so analogWrite() on those is dead.
//    Pin 3 still works as a plain digital output (DIR) - we never
//    enable COM3C. millis()/delay() are on Timer0 and are untouched.
//
//  * ENA polarity: on most DM860H units the ENA opto DISABLES the
//    motor when energised. If the shaft is free-spinning, try HIGH,
//    or simply leave ENA unwired - the driver defaults to enabled.
//
//  * If the motor screams and does not turn, lower V_MAX first, then
//    ACCEL. If it turns but stalls under load, drop V_MAX.
//
//  * Keep V_MAX below ~100 kHz. Above that the falling-edge ISR gets
//    tight against the next rising edge and the step count can slip.
//======================================================================