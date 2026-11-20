#include <Arduino.h>

// Single Metal Generator, AVR16DD14 / AVR32DD14
// Pins: p1=PD4, p2=PD5, p3=PD7, DAC out=PD6
//       trigger=PA1 (active low), LED=PA0 (PWM, active low)
//       amp CV=PC1, dec pot=PC2, mode switch=PC3
// Settings: 24 MHz internal, millis timer NOT on TCB0
//
// Mode 1 (switch to GND)   : 3 independent oscillators, amp = volume
// Mode 2 (switch centre)   : p1 = tune, p2 = detune, amp jack = 1V/oct
//                            (5 V = no shift, 0 V = 5 octaves down), volume full
// Mode 3 (switch to 5 V)   : same as mode 1 for now

#define TICK_HZ 100000UL

// ---- tuning ----
#define FMIN  40.0f      // Hz, pot fully counter-clockwise
#define FMAX  800.0f     // Hz, pot fully clockwise

// 0 = XOR of all three, 1/2/3 = only that oscillator (for testing)
#define SOLO 0

#define INC_MIN ((uint32_t)(FMIN * 4294967296.0f / TICK_HZ))

// ---- lookup tables, filled once in setup() ----
static uint32_t octTbl[65];   // 2^(i/64) - 1, scaled by 65536
static uint16_t decTbl[65];   // decay coefficient per ms, 16-bit fraction
static uint32_t octQ;         // octaves * 1024 per full pot travel

// ---- shared with ISR ----
volatile uint32_t inc1, inc2, inc3;   // phase increments (frequency)
volatile uint16_t decayCoef = 65000;  // envelope decay per ms (16-bit fraction)
volatile uint16_t ampIn     = 1023;   // volume, 0..1023
volatile uint16_t amp16     = 0;      // output swing in 1/16 DAC steps, 0..8183
volatile uint8_t  ledLevel  = 0;      // envelope brightness 0..255

static uint32_t ph1, ph2, ph3;
static uint16_t env;                  // 0..65535
static uint16_t rnd = 0xACE1;         // dither noise
static uint8_t  slowDiv, lastTin = 1;

ISR(TCB0_INT_vect) {
  TCB0.INTFLAGS = TCB_CAPT_bm;

  // trigger is checked every tick (10 us), envelope starts immediately
  uint8_t t = VPORTA.IN & PIN1_bm;                    // active low
  if (!t && lastTin) {
    env = 65535;
    amp16 = ((uint32_t)65535 * ampIn) >> 13;
  }
  lastTin = t;

  ph1 += inc1; ph2 += inc2; ph3 += inc3;

#if SOLO == 1
  uint8_t bit = (uint8_t)(ph1 >> 31);
#elif SOLO == 2
  uint8_t bit = (uint8_t)(ph2 >> 31);
#elif SOLO == 3
  uint8_t bit = (uint8_t)(ph3 >> 31);
#else
  uint8_t bit = (uint8_t)((ph1 ^ ph2 ^ ph3) >> 31);   // XOR of the 3 squares
#endif

  // cheap noise for dithering the amplitude
  rnd = rnd * 25173 + 13849;
  uint16_t a = (amp16 + (rnd >> 12)) >> 4;            // dithered swing, 0..512
  if (a > 511) a = 511;

  DAC0.DATA = (uint16_t)(bit ? (512 + a) : (512 - a)) << 6;   // 512 = silence

  if (++slowDiv >= 100) {             // 1 kHz envelope tick
    slowDiv = 0;

    uint16_t n = ((uint32_t)env * decayCoef) >> 16;   // exponential decay
    if (n == env && env) n--;                         // don't stall above zero
    env = n;

    amp16 = ((uint32_t)env * ampIn) >> 13;            // env * volume -> 0..8183
    ledLevel = env >> 8;                              // LED follows the envelope
  }
}

// pot reading 0..1023 -> phase increment, exponential FMIN..FMAX, integer only
static uint32_t potToInc(uint16_t raw) {
  uint32_t o = ((uint32_t)raw * octQ) >> 10;          // octaves in 1/1024 units
  uint8_t  n = o >> 10;                               // whole octaves
  uint16_t f = o & 1023;                              // fraction of an octave
  uint8_t  idx = f >> 4, rem = f & 15;
  uint32_t t0 = octTbl[idx], t1 = octTbl[idx + 1];
  uint32_t v16 = t0 + (((t1 - t0) * rem) >> 4);       // 0..65535
  uint32_t base = INC_MIN << n;
  return base + (base >> 8) * (v16 >> 8);
}

// dec pot reading 0..1023 -> decay coefficient (5 ms .. 2 s)
static uint16_t potToDecay(uint16_t raw) {
  uint8_t idx = raw >> 4, rem = raw & 15;
  int32_t t0 = decTbl[idx], t1 = decTbl[idx + 1];
  return (uint16_t)(t0 + (((t1 - t0) * rem) >> 4));
}

// lower a phase increment by (5V - CV) octaves. raw 1023 = 5 V = no shift
static uint32_t shiftDown(uint32_t base, uint16_t raw) {
  uint32_t o = ((uint32_t)(1023 - raw) * 5120UL) / 1023;   // octaves in 1/1024 units
  uint8_t  n = o >> 10;                                    // whole octaves down
  uint16_t f = o & 1023;                                   // fraction of an octave
  uint32_t factor = 65536;                                 // 1.0
  if (f) {
    uint16_t g = 1024 - f;
    uint8_t idx = g >> 4, rem = g & 15;
    uint32_t t0 = octTbl[idx], t1 = octTbl[idx + 1];
    uint32_t v = t0 + (((t1 - t0) * rem) >> 4);            // 2^(g/1024) - 1
    factor = (65536UL + v) >> 1;                           // 2^(-f/1024)
  }
  return (((base >> 8) * (factor >> 8))) >> n;
}

void setup() {
  analogReference(VDD);
  VREF.ADC0REF = VREF_REFSEL_VDD_gc;
  analogReadResolution(10);

  // build tables once (float is fine here)
  octQ = (uint32_t)(logf(FMAX / FMIN) / 0.693147f * 1024.0f * 1024.0f / 1023.0f + 0.5f);
  for (uint8_t i = 0; i <= 64; i++) {
    octTbl[i] = (uint32_t)(65536.0f * (powf(2.0f, i / 64.0f) - 1.0f) + 0.5f);
    float tau = 0.005f * powf(400.0f, i / 64.0f);
    float c = 65536.0f * expf(-0.001f / tau);
    decTbl[i] = (c > 65535.0f) ? 65535 : (uint16_t)c;
  }

  // trigger on PA1; the LED on PA0 is driven by analogWrite()
  PORTA.DIRCLR = PIN1_bm;
  PORTA.PIN1CTRL = PORT_PULLUPEN_bm;
  analogWrite(PIN_PA0, 255);          // LED off (active low)

  // DAC on PD6, VDD reference
  VREF.DAC0REF = VREF_REFSEL_VDD_gc;
  DAC0.CTRLA   = DAC_ENABLE_bm | DAC_OUTEN_bm;
  DAC0.DATA    = 512 << 6;

  // 100 kHz tick
  TCB0.CTRLB   = TCB_CNTMODE_INT_gc;
  TCB0.CCMP    = (F_CPU / TICK_HZ) - 1;
  TCB0.INTCTRL = TCB_CAPT_bm;
  TCB0.CTRLA   = TCB_CLKSEL_DIV1_gc | TCB_ENABLE_bm;
  sei();
}

void loop() {
  static uint8_t  slow = 0;
  static uint8_t  mode = 1;
  static uint16_t p2 = 0, p3 = 0, coef = 65000;

  if ((slow++ & 15) == 0) {                            // slow controls
    int m = analogRead(PIN_PC3);
    mode = (m < 256) ? 1 : (m < 768) ? 2 : 3;
    p2   = analogRead(PIN_PD5);
    p3   = analogRead(PIN_PD7);
    coef = potToDecay(analogRead(PIN_PC2));
  }

  uint16_t p1 = analogRead(PIN_PD4);
  (void)analogRead(PIN_PC1);                           // discard: settles the 100k source
  uint16_t cv = analogRead(PIN_PC1);

  uint32_t i1, i2, i3;
  uint16_t amp;

  if (mode == 2) {
    // p1 = tune (pitch at 5 V), amp jack = 1V/oct CV, p2 = detune
    i1 = shiftDown(potToInc(p1), cv);
    uint32_t spread = (i1 >> 11) * p2;
    i2 = i1 + spread;
    i3 = i1 - spread;
    amp = 1023;                                        // volume fixed at full
  } else {
    // modes 1 and 3: three independent oscillators, amp = volume
    i1 = potToInc(p1);
    i2 = potToInc(p2);
    i3 = potToInc(p3);
    amp = cv;
  }

  cli();
  inc1 = i1; inc2 = i2; inc3 = i3;
  decayCoef = coef;
  ampIn = amp;
  sei();

  // LED fades with the envelope (active low, so invert)
  static uint8_t lastLed = 0;
  uint8_t lv = ledLevel;
  if (lv != lastLed) {
    lastLed = lv;
    analogWrite(PIN_PA0, 255 - lv);
  }
}
