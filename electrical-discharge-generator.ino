
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <avr/io.h>
#include <stdint.h>
#include <string.h>

// LCD: RS=D4, E=D9, D4=D8, D5=D7, D6=D6, D7=D5
LiquidCrystal lcd(4, 9, 8, 7, 6, 5);

// ===== MCP23017 =====
#define MCP_ADDR  0x20
#define IODIRA    0x00
#define IODIRB    0x01
#define GPPUB     0x0D
#define OLATA     0x14
#define OLATB     0x15

static uint8_t mcpA = 0, mcpB = 0;
static uint8_t mcpA_sent = 0xFF, mcpB_sent = 0xFF;

static uint8_t mcpWriteReg(uint8_t reg, uint8_t val) {
  for (uint8_t attempt = 0; attempt < 2; attempt++) {
    Wire.beginTransmission(MCP_ADDR);
    Wire.write(reg);
    Wire.write(val);
    uint8_t st = Wire.endTransmission();
    if (st == 0) return 0;
    delay(2);
  }
  return 1;
}

static void mcpCommit() {
  if (mcpA != mcpA_sent) {
    if (mcpWriteReg(OLATA, mcpA) == 0) mcpA_sent = mcpA;
  }
  if (mcpB != mcpB_sent) {
    if (mcpWriteReg(OLATB, mcpB) == 0) mcpB_sent = mcpB;
  }
}

static inline void bitSetTo(uint8_t &x, uint8_t b, bool on) {
  if (on) x |= (1U << b); else x &= ~(1U << b);
}

// Obr: GPB5..GPB7
static inline void setObr(bool d, bool z, bool c) {
  bitSetTo(mcpB, 5, d);
  bitSetTo(mcpB, 6, z);
  bitSetTo(mcpB, 7, c);
}

// HV_OFF_D LED: GPB2 (jeśli u Ciebie LED jest active-LOW, odwróć na !on)
static inline void setHvOffLed(bool on) { bitSetTo(mcpB, 2, on); }

// Elektrody: GPB3=EL+, GPB4=EL-
static inline void setElBits(bool plus) {
  bitSetTo(mcpB, 3, plus);
  bitSetTo(mcpB, 4, !plus);
}

// ===== R/C odwrócone: "wszystkie świecą poza wybraną" =====
#define R_MASK_A  0x1F  // GPA0..GPA4
#define C_MASK_A  0xE0  // GPA5..GPA7
#define C_MASK_B  0x03  // GPB0..GPB1

static uint8_t screen = 1;
static uint8_t selR = 2, selC = 2;

static inline void applyRSelectionInverted() {
  mcpA |= R_MASK_A;
  mcpA &= ~(1U << (selR - 1));
}

static inline void applyCSelectionInverted() {
  mcpA |= C_MASK_A;
  mcpB |= C_MASK_B;
  if (selC <= 3) mcpA &= ~(1U << (4 + selC));  // C1->5, C2->6, C3->7
  else           mcpB &= ~(1U << (selC - 4));  // C4->0, C5->1
}

// ===== Elektrody + diody na Arduino =====
static bool elPlus = true;
static inline void applyElectrode(bool plus) {
  elPlus = plus;
  setElBits(elPlus);

  // D2 = EL+D, D1 = EL-D (ACTIVE HIGH)
  if (elPlus) { PORTD |=  (1 << PD2); PORTD &= ~(1 << PD1); }
  else        { PORTD &= ~(1 << PD2); PORTD |=  (1 << PD1); }
}

// ===== ADC (bez analogRead) =====
static inline void adcInit() {
  ADMUX = (1 << REFS0);
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}
static inline uint16_t adcRead(uint8_t ch) {
  ADMUX = (1 << REFS0) | (ch & 0x07);

  ADCSRA |= (1 << ADSC); while (ADCSRA & (1 << ADSC)) {} // dummy
  ADCSRA |= (1 << ADSC); while (ADCSRA & (1 << ADSC)) {} // real

  return ADC;
}

// ===== Wejścia ACTIVE LOW =====
static inline bool hvOffRead() { return ((PINB & (1 << PB4)) == 0); } // D12
static inline bool crRead()    { return ((PINC & (1 << PC3)) == 0); } // A3
static inline uint8_t winRaw() { return (PIND & (1 << PD0)) ? 1 : 0; } // D0
static inline uint8_t ustRaw() { return (PIND & (1 << PD3)) ? 1 : 0; } // D3

// ===== Stan =====
static bool hvOff = false;
static bool hvOffD = false;

static int32_t V_mV = 0;     // pomiar napięcia (mV)
static int32_t I_mA = 0;     // pomiar prądu (mA)
static int32_t V_hold = 28;  // V Ustalona (V) - do obliczeń LED

static uint16_t pot_adc = 0; // A0 do wyboru 1..5 na ekranie 3

// ===== Debounce =====
static uint8_t lastWin = 1, lastUst = 1;
static inline bool fellWin() {
  uint8_t r = winRaw();
  if (lastWin && !r) { delay(20); if (!winRaw()) { lastWin = 0; return true; } }
  if (r) lastWin = 1;
  return false;
}
static inline bool fellUst() {
  uint8_t r = ustRaw();
  if (lastUst && !r) { delay(20); if (!ustRaw()) { lastUst = 0; return true; } }
  if (r) lastUst = 1;
  return false;
}

static inline uint8_t potSel1to5() {
  uint8_t s = 1 + (uint8_t)((pot_adc * 5UL) / 1024UL);
  if (s < 1) s = 1;
  if (s > 5) s = 5;
  return s;
}

// ===== Pomiary =====
// Napięcie: 5 próbek co 40ms -> ~200ms
static uint8_t nV = 0;
static uint32_t accV = 0;

// Prąd: 10 próbek co 40ms -> ~400ms, wybierz top3 i uśrednij
static uint8_t nI = 0;
static uint16_t imax1 = 0, imax2 = 0, imax3 = 0;

// Takt 40ms
static unsigned long tSample = 0;

static inline void top3_update(uint16_t x) {
  if (x >= imax1) { imax3 = imax2; imax2 = imax1; imax1 = x; }
  else if (x >= imax2) { imax3 = imax2; imax2 = x; }
  else if (x > imax3)  { imax3 = x; }
}

static void measurementTick() {
  unsigned long now = millis();
  if ((uint32_t)(now - tSample) < 40) return;
  tSample = now; // bez "nadganiania" po delay

  // A0 do wyboru 1..5 (ekran 3)
  pot_adc = adcRead(0);

  // próbki V i I
  uint16_t vRaw = adcRead(1);
  uint16_t iRaw = adcRead(2);

  // --- napięcie (5 próbek) ---
  accV += vRaw;
  if (++nV >= 5) {
    uint32_t vAvg = accV / 5;
    accV = 0;
    nV = 0;

    // V: 0..~5V -> 0..205V (jak wcześniej)
    V_mV = (int32_t)((vAvg * 205000UL) / 1023UL);

    // hvOffD: HV_OFF ON i V<=30V
    hvOffD = (hvOff && (V_mV <= 30000));
    setHvOffLed(hvOffD);

    // LED D/Z/C wg nowych progów w VOLTACH
    if (hvOff) {
      setObr(false, false, false);
    } else {
      int32_t V_meas_V = (V_mV + 500) / 1000; // pełne V, tak jak na górze ekranu 1
      if (V_meas_V < 0) V_meas_V = 0;

      // progi: czerwony gdy 10*V_meas <= 1*V_hold
      // zielony gdy 1*V_hold < 10*V_meas < 8*V_hold
      // pomarańcz gdy 10*V_meas >= 8*V_hold
      int32_t lhs = 10 * V_meas_V;
      int32_t t10 = 1 * V_hold;
      int32_t t80 = 8 * V_hold;

      if (lhs <= t10) {
        // zwarcie (czerwony)
        setObr(false, false, true);
      } else if (lhs < t80) {
        // obróbka (zielony)
        setObr(false, true, false);
      } else {
        // za daleko (pomarańczowy)
        setObr(true, false, false);
      }
    }

    mcpCommit();
  }

  // --- prąd (10 próbek, top3) ---
  top3_update(iRaw);
  if (++nI >= 10) {
    uint16_t avgTop3 = (uint16_t)(( (uint32_t)imax1 + (uint32_t)imax2 + (uint32_t)imax3 ) / 3U);

    // SHUNT = 5 ohm (2x10 równolegle)
    int32_t vsh_mV = (int32_t)(( (uint32_t)avgTop3 * 5000UL ) / 1023UL);
    I_mA = vsh_mV / 5;

    // reset okna
    nI = 0;
    imax1 = imax2 = imax3 = 0;
  }
}

// ===== HV_OFF switch =====
static void handleHvOff() {
  bool r = hvOffRead();
  if (r == hvOff) return;
  delay(10);
  r = hvOffRead();
  if (r == hvOff) return;

  hvOff = r;

  if (hvOff) {
    V_hold = 28;
    setObr(false, false, false);
    hvOffD = (V_mV <= 30000);
    setHvOffLed(hvOffD);
  } else {
    hvOffD = false;
    setHvOffLed(false);
  }
  mcpCommit();
}

// ===== Ekrany + Ustaw =====
static void handleWindow() {
  if (fellWin()) { if (++screen > 4) screen = 1; }
}

static void handleUstaw() {
  if (!fellUst()) return;

  // ekran 1: zapis aktualnego napięcia (z A1) jako V_hold
  if (screen == 1) {
    if (!hvOff) {
      int32_t V_now = (V_mV + 500) / 1000; // round
      if (V_now < 0) V_now = 0;
      if (V_now > 205) V_now = 205;
      V_hold = V_now;
    }
    return;
  }

  // ekran 3: C/R tylko gdy hvOffD
  if (screen == 3) {
    if (!hvOffD) return;
    uint8_t s = potSel1to5();

    if (crRead()) {
      if (s != selR) {
        // stara wybrana ma się zapalić
        mcpA |= R_MASK_A;
        mcpCommit();
        delay(500);

        // nowa wybrana ma zgasnąć
        selR = s;
        applyRSelectionInverted();
        mcpCommit();
      }
    } else {
      if (s != selC) {
        // stara wybrana ma się zapalić
        mcpA |= C_MASK_A;
        mcpB |= C_MASK_B;
        mcpCommit();
        delay(500);

        // nowa wybrana ma zgasnąć
        selC = s;
        applyCSelectionInverted();
        mcpCommit();
      }
    }
    return;
  }

  // ekran 4: elektroda tylko gdy hvOffD
  if (screen == 4) {
    if (!hvOffD) return;
    setElBits(false);
    PORTD &= ~((1 << PD1) | (1 << PD2));
    mcpCommit();
    delay(500);
    applyElectrode(!elPlus);
    mcpCommit();
  }
}

// ===== LCD bez migotania =====
static char last0[17] = {0};
static char last1[17] = {0};

static inline void fill16(char *s) { for (uint8_t i=0;i<16;i++) s[i]=' '; s[16]=0; }
static inline void putStr(char *dst, uint8_t pos, const char *s) { while (*s && pos < 16) dst[pos++] = *s++; }

static inline void put3V(char *dst, uint8_t pos, int32_t v) {
  if (v < 0) v = 0; if (v > 999) v = 999;
  dst[pos+0] = (v >= 100) ? ('0' + (v/100)) : ' ';
  dst[pos+1] = (v >= 10)  ? ('0' + ((v/10)%10)) : ' ';
  dst[pos+2] = '0' + (v%10);
  dst[pos+3] = 'V';
}

static inline void putVmeas(char *dst, uint8_t pos, int32_t VmV) {
  if (VmV < 0) VmV = 0;
  int32_t dv = (VmV + 50) / 100; // 0.1V
  int32_t whole = dv/10;
  int32_t frac  = dv%10;
  if (whole > 999) whole = 999;
  dst[pos+0] = (whole>=100) ? ('0'+(whole/100)) : ' ';
  dst[pos+1] = (whole>=10)  ? ('0'+((whole/10)%10)) : ' ';
  dst[pos+2] = '0'+(whole%10);
  dst[pos+3] = '.';
  dst[pos+4] = '0'+(uint8_t)frac;
  dst[pos+5] = 'V';
}

static inline void putImeas(char *dst, uint8_t pos, int32_t ImA) {
  if (ImA < 0) ImA = 0;
  int32_t a100 = (ImA + 5) / 10;  // 0.01A
  int32_t whole = a100 / 100;     // 0..9
  int32_t frac2 = a100 % 100;
  if (whole > 9) whole = 9;
  dst[pos+0] = '0' + (uint8_t)whole;
  dst[pos+1] = '.';
  dst[pos+2] = '0' + (uint8_t)(frac2/10);
  dst[pos+3] = '0' + (uint8_t)(frac2%10);
  dst[pos+4] = 'A';
}

static void renderLCD() {
  char l0[17], l1[17];
  fill16(l0); fill16(l1);

  if (screen == 1) {
    int32_t V_now = (V_mV + 500) / 1000;
    if (V_now < 0) V_now = 0;
    if (V_now > 205) V_now = 205;

    put3V(l0, 0, V_now);
    putStr(l0, 6, "V Ustal");

    put3V(l1, 0, V_hold);
    putStr(l1, 6, "V Ustalona");
  }
  else if (screen == 2) {
    putVmeas(l0, 0, V_mV);
    putStr(l0, 6, "V Napiecie");
    putImeas(l1, 0, I_mA);
    putStr(l1, 6, "I prad");
  }
  else if (screen == 3) {
    putStr(l0, 0, "Ustaw C na:");
    putStr(l1, 0, "Ustaw R na:");
    uint8_t s = potSel1to5();
    bool cr = crRead();
    uint8_t top = cr ? selC : s;
    uint8_t bot = cr ? s    : selR;
    l0[14] = '0' + top;
    l1[14] = '0' + bot;
    if (!hvOffD) { l0[15] = '!'; l1[15] = '!'; }
  }
  else {
    putStr(l0, 0, "Wybor Elektrody");
    putStr(l1, 6, elPlus ? "EL+" : "EL-");
    if (!hvOffD) { l0[15] = '!'; l1[15] = '!'; }
  }

  if (memcmp(l0, last0, 16) != 0) {
    memcpy(last0, l0, 17);
    lcd.setCursor(0,0);
    lcd.print(l0);
  }
  if (memcmp(l1, last1, 16) != 0) {
    memcpy(last1, l1, 17);
    lcd.setCursor(0,1);
    lcd.print(l1);
  }
}

void setup() {
  // D1,D2 outputs (EL-D, EL+D)
  DDRD |= (1 << DDD1) | (1 << DDD2);

  // D0,D3 input pull-up
  DDRD &= ~((1 << DDD0) | (1 << DDD3));
  PORTD |= (1 << PD0) | (1 << PD3);

  // D12 (PB4) input pull-up
  DDRB &= ~(1 << DDB4);
  PORTB |= (1 << PB4);

  // A3 (PC3) input pull-up
  DDRC &= ~(1 << DDC3);
  PORTC |= (1 << PC3);

  adcInit();

  Wire.begin();
  Wire.setClock(50000);
  delay(20);

  lcd.begin(16, 2);
  delay(30);
  lcd.clear();

  // MCP23017: latch -> IODIR
  mcpA = 0;
  mcpB = 0;

  // start: wybrana zgaszona, reszta świeci
  selR = 2; selC = 2;
  applyRSelectionInverted();
  applyCSelectionInverted();

  setObr(false,false,false);
  setHvOffLed(false);
  applyElectrode(true);

  mcpA_sent = 0xFF;
  mcpB_sent = 0xFF;

  mcpWriteReg(OLATA, mcpA);
  mcpWriteReg(OLATB, mcpB);

  mcpWriteReg(IODIRA, 0x00);
  mcpWriteReg(IODIRB, 0x00);
  mcpWriteReg(GPPUB,  0x00);

  mcpCommit();

  hvOff = hvOffRead();
  tSample = millis();

  for (uint8_t i=0;i<16;i++){ last0[i]=0; last1[i]=0; }
  renderLCD();
}

void loop() {
  handleHvOff();
  measurementTick();
  handleWindow();
  handleUstaw();
  renderLCD();
}
