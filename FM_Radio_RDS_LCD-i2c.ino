/*
 * =====================================================================
 *  Radio FM z RDS:  Arduino Nano + RDA5807M + LCD 20x4 (HD44780, I2C)
 * =====================================================================
 *  Na LCD: częstotliwość, PS, RadioText, PI Code, PTY
 *  (dodatkowo: głośność, stereo, wskaźnik RDS i siła sygnału).
 *
 *  UKŁAD EKRANU (20 znaków x 4 wiersze)
 *    1: 100.05MHz V:8   ST  #     częstotliwość, głośność, stereo, siła sygnału
 *    2: RMF FM   PI:3C7F RDS      PS, PI Code, wskaźnik RDS
 *    3: PTY10:Pop Music           PTY
 *    4: (linia RadioText)         RadioText
 *
 *  JAK DZIAŁA RDS
 *   - PS, PI, PTY pojawiają się od razu po odebraniu pierwszej poprawnej grupy.
 *   - RadioText przy pierwszym wczytaniu pokazuje się "na żywo" (znaki w miarę
 *     odbioru). Potem tekst jest TRZYMANY na ekranie i podmieniany dopiero, gdy
 *     nowa treść zostanie odebrana w całości (bez migania i bez mieszania
 *     starego z nowym). Zanik sygnału nie kasuje tekstu.
 *   - RadioText (do 64 znaków) nie mieści się w jednym wierszu LCD 20x4, więc
 *     jest dzielony na linie po 20 znaków i pokazywany linia po linii
 *     (zmiana co RT_PAGE_MS).
 *   - Wszystko kasuje się dopiero po zmianie stacji.
 *
 *  BIBLIOTEKI (Menedżer bibliotek Arduino IDE):
 *    - "LiquidCrystal I2C" (Frank de Brabander)
 *  RDA5807M obsługiwany jest bezpośrednio przez Wire, a dekoder RDS
 *  (PS / RT / PI / PTY) jest zaimplementowany w tym szkicu.
 *
 *  PODŁĄCZENIE (wszystko na jednej magistrali I2C)
 *    Nano A4 (SDA) -> SDA LCD   i  SDA RDA5807M
 *    Nano A5 (SCL) -> SCL LCD   i  SCL RDA5807M
 *    RDA5807M: VCC = 3,3 V (układ 2,7–3,6 V!), GND = GND
 *    LCD:      VCC = 5 V (typowy LCD 20x4 z konwerterem PCF8574), GND = GND
 *              UWAGA: konwerter I2C LCD zwykle ma pull-upy SDA/SCL do VCC
 *              (5 V), a RDA5807M jest 3,3 V - w razie wątpliwości użyj
 *              konwertera poziomów I2C.
 *              Adres I2C LCD: najczęściej 0x27 (czasem 0x3F) - patrz LCD_I2C_ADDR.
 *    Antena:   ok. 75 cm przewodu na pad ANT lub przewód słuchawek
 *    Audio:    wyjście słuchawkowe modułu (L/R/GND) -> słuchawki / wzmacniacz
 *
 *  PRZYCISKI (jeden koniec do pinu, drugi do GND, pull-up wewnętrzny)
 *    D8  - częstotliwość w górę : krótko = +0,05 MHz, długo = szukaj stacji w górę
 *    D9  - częstotliwość w dół  : krótko = -0,05 MHz, długo = szukaj stacji w dół
 *    D10  - głośność +
 *    D11  - głośność -
 *
 *  Ostatnia stacja i głośność są zapamiętywane w EEPROM.
 *
 *  UWAGA: jeśli ekran świeci, ale nie widać znaków - ustaw kontrast
 *  potencjometrem na tylnej płytce konwertera I2C.
 * =====================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>

// ============================ KONFIGURACJA ============================
#define PIN_FREQ_UP      8
#define PIN_FREQ_DOWN    9
#define PIN_VOL_UP       10
#define PIN_VOL_DOWN     11

#define FREQ_MIN         8750    // 87,50 MHz (jednostka = 10 kHz)
#define FREQ_MAX         10800   // 108,00 MHz
#define FREQ_DEFAULT     10000   // 100,00 MHz
#define FREQ_STEP        5       // krok strojenia: 0,05 MHz
#define VOLUME_DEFAULT   8       // 0..15
#define LONG_PRESS_MS    600

// Maksymalny poziom błędów bloku RDS akceptowany przez dekoder:
//   0 = tylko bloki bez błędów (najmniej "literówek", wolniej przy słabym sygnale)
//   1 = także bloki poprawione przez układ (szybciej, domyślnie)
//   3 = bez filtrowania
#define RDS_MAX_BLER     1

// 1 = wyłącz wewnętrzne pull-upy Nano (5 V) na SDA/SCL. Włącz, jeśli
// moduły mają własne pull-upy do 3,3 V (typowe dla modułów RDA5807M).
#define DISABLE_INTERNAL_PULLUPS 0

// Wyświetlacz LCD 20x4 (HD44780 + konwerter I2C PCF8574)
#define LCD_I2C_ADDR     0x27    // jeśli ekran nic nie pokazuje, spróbuj 0x3F
#define LCD_WIDTH        20
#define LCD_HEIGHT       4
#define RT_LINE_CHARS    20                   // znaków w linii RadioText
#define RT_LINES         4                    // 4 x 20 = 80 >= 64 znaki RT
#define RT_PAGE_MS       2500                 // co ile ms zmienia się linia RT

LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_WIDTH, LCD_HEIGHT);

// ============================== RDA5807M ==============================
#define RDA_ADDR_SEQ     0x10    // dostęp sekwencyjny (odczyt od rejestru 0x0A)
#define RDA_ADDR_REG     0x11    // dostęp do pojedynczego rejestru

#define REG_CONTROL      0x02
#define REG_CHANNEL      0x03
#define REG_SYSCFG       0x04
#define REG_VOLUME       0x05

// 0x02: DHIZ=1 DMUTE=1 | RDS_EN=1 NEW_METHOD=1 ENABLE=1
#define CTRL_BASE        0xC00D
#define CTRL_MUTE_BIT    0x4000
#define CTRL_MUTED       ((uint16_t)(CTRL_BASE & ~CTRL_MUTE_BIT))
#define CTRL_SEEK        0x0100
#define CTRL_SEEKUP      0x0200

// =============================== STAN =================================
uint16_t freq100 = FREQ_DEFAULT;
uint8_t  volume = VOLUME_DEFAULT;
uint16_t reg[6];                 // rejestry 0x0A..0x0F

bool     stereo      = false;
uint8_t  rssiBars    = 0;
bool     rdsActive   = false;
uint32_t lastGroupMs = 0;
bool     dirty       = true;     // trzeba przerysować ekran

bool     settingsPending = false;
uint32_t settingsMs      = 0;

// ---- dane RDS ----
uint16_t piOut  = 0;  bool piOk  = false;
uint8_t  ptyOut = 0;  bool ptyOk = false;
char     psTxt[9];                       // PS (wyświetlany)

// RadioText: bufor roboczy (rtStage) i tekst wyświetlany (rtTxt)
char     rtStage[65];
char     rtTxt[65];
bool     rtShown = false;                // czy RT został już raz zatwierdzony
uint16_t rtMask  = 0;                    // bit n = odebrano segment o adresie n
uint8_t  rtEnd   = 64;                   // pozycja znaku 0x0D (64 = nieznana)
bool     rtAB = false, rtABvalid = false;
uint8_t  rtPrevAddr = 0, rtPassMax = 0, rtPassMaxPrev = 0xFF;
uint8_t  rtPage = 0;                     // która linia RT jest teraz na ekranie
uint8_t  rtLineCount = 0;                // ile linii ma aktualny RT
uint32_t rtPageMs = 0;                   // kiedy ostatnio zmieniła się linia RT

uint16_t lastA = 0, lastB = 0, lastC = 0, lastD = 0;   // do odrzucania duplikatów

// ---- teksty/dane do rysowania (kopia robocza na czas jednej klatki) ----
char sFreq[8], sVol[6], sPs[9], sPi[10], sPty[26];
char rtL[RT_LINES][RT_LINE_CHARS + 1];
bool dStereo = false, dRds = false;
uint8_t dBars = 0;

// ---- przyciski ----
enum { EV_NONE = 0, EV_SHORT = 1, EV_LONG = 2 };
struct Btn { uint8_t pin; bool down; bool longSent; uint32_t t0; };
Btn btnUp   = { PIN_FREQ_UP,   false, false, 0 };
Btn btnDown = { PIN_FREQ_DOWN, false, false, 0 };
Btn btnVUp  = { PIN_VOL_UP,    false, false, 0 };
Btn btnVDn  = { PIN_VOL_DOWN,  false, false, 0 };

// Nazwy PTY (RDS, Europa) - 32 pozycje oddzielone znakiem '|'
const char PTY_NAMES[] PROGMEM =
  "No PTY|News|Affairs|Info|Sport|Education|Drama|Culture|Science|Varied|"
  "Pop Music|Rock Music|Easy Listening|Light Classic|Classical|Other Music|"
  "Weather|Finance|Children|Social|Religion|Phone In|Travel|Leisure|"
  "Jazz Music|Country Music|National Music|Oldies Music|Folk Music|"
  "Documentary|Alarm Test|ALARM!";

// ============================ I2C: RDA5807M ===========================
void rdaWrite(uint8_t r, uint16_t val) {
  Wire.beginTransmission((uint8_t)RDA_ADDR_REG);
  Wire.write(r);
  Wire.write((uint8_t)(val >> 8));
  Wire.write((uint8_t)(val & 0xFF));
  Wire.endTransmission();
}

// Czyta rejestry 0x0A..0x0F do reg[0..5]
bool readRegs() {
  if (Wire.requestFrom((uint8_t)RDA_ADDR_SEQ, (uint8_t)12) != 12) return false;
  for (uint8_t i = 0; i < 6; i++) {
    uint8_t hi = Wire.read();     // kolejność odczytu musi być jawna
    uint8_t lo = Wire.read();
    reg[i] = ((uint16_t)hi << 8) | lo;
  }
  return true;
}

bool waitStc(uint16_t timeoutMs) {            // czekaj na koniec strojenia/szukania
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    if (readRegs() && (reg[0] & 0x4000)) return true;   // STC
    delay(10);
  }
  return false;
}

// ================================ RDS =================================
// Kasowanie WSZYSTKICH danych RDS - tylko przy zmianie stacji
void resetRds() {
  strcpy(psTxt, "--------");
  memset(rtStage, ' ', 64);  rtStage[64] = 0;
  memset(rtTxt,   ' ', 64);  rtTxt[64]   = 0;
  rtShown = false;
  rtMask = 0; rtEnd = 64; rtABvalid = false;
  rtPrevAddr = 0; rtPassMax = 0; rtPassMaxPrev = 0xFF;
  rtPage = 0; rtLineCount = 0; rtPageMs = millis();
  piOk = false; ptyOk = false;
  lastA = lastB = lastC = lastD = 0;
  lastGroupMs = 0;
  rdsActive = false;
  dirty = true;
}

// Znak RDS -> ASCII (LCD nie ma polskich liter)
char cleanChar(uint8_t ch) {
  if (ch >= 0x20 && ch <= 0x7E) return (char)ch;
  if (ch >= 0x80) return '?';
  return ' ';
}

// ---- PS: znaki wpisywane od razu ----
void putPs(uint8_t pos, uint8_t ch) {
  if (pos >= 8) return;
  char c = cleanChar(ch);
  if (psTxt[pos] != c) { psTxt[pos] = c; dirty = true; }
}

// ---- RadioText ----
// Zatwierdza tekst z bufora roboczego na ekran, jeśli odebrano segmenty 0..lastSeg.
// cut = pozycja końca tekstu (64 = cały bufor).
void rtCommit(uint8_t lastSeg, uint8_t cut) {
  uint16_t need = (lastSeg >= 15) ? 0xFFFF : (uint16_t)((1u << (lastSeg + 1)) - 1);
  if ((rtMask & need) != need) return;          // jeszcze nie wszystko odebrane

  uint8_t n = (cut < 64) ? (uint8_t)(cut + 1) : 64;
  bool diff = false;
  for (uint8_t i = 0; i < n; i++) {
    char ch = (i == cut) ? '\r' : rtStage[i];   // '\r' = koniec tekstu
    if (rtTxt[i] != ch) { rtTxt[i] = ch; diff = true; }
  }
  rtShown = true;
  if (diff) { dirty = true; rtPage = 0; rtPageMs = millis(); }   // nowy tekst - od 1. linii
}

// Zapisuje jeden segment (2 lub 4 znaki) do bufora roboczego
void rtStoreSegment(uint8_t addr, uint8_t segLen, const uint8_t *raw) {
  uint8_t base = addr * segLen;
  char cc[4];
  for (uint8_t i = 0; i < segLen; i++) cc[i] = (raw[i] == 0x0D) ? '\r' : cleanChar(raw[i]);

  // Ten segment był już odebrany, a treść jest inna => nadawana jest nowa
  // wiadomość (lub błąd transmisji): zbieraj od nowa, ale NIE ruszaj ekranu.
  if (rtMask & (1u << addr)) {
    bool changed = false;
    for (uint8_t i = 0; i < segLen; i++) if (rtStage[base + i] != cc[i]) changed = true;
    if (changed) { rtMask = 0; rtEnd = 64; }
  }

  for (uint8_t i = 0; i < segLen; i++) {
    rtStage[base + i] = cc[i];
    if (cc[i] == '\r' && (uint8_t)(base + i) < rtEnd) rtEnd = base + i;
  }
  rtMask |= (1u << addr);

  // Pierwsze wczytanie po zmianie stacji: pokazuj znaki od razu, w miarę odbioru
  if (!rtShown) {
    for (uint8_t i = 0; i < segLen; i++) rtTxt[base + i] = cc[i];
    dirty = true;
  }
}

void decodeGroup(uint16_t a, uint16_t b, uint16_t c, uint16_t d, uint8_t blerA) {
  uint8_t groupType = b >> 12;
  bool    versionB  = (b >> 11) & 1;
  uint8_t pty       = (b >> 5) & 0x1F;
  bool    okA       = (blerA <= RDS_MAX_BLER);

  // --- PI (blok A) - od razu ---
  if (okA && a != 0 && (!piOk || piOut != a)) { piOut = a; piOk = true; dirty = true; }

  // --- PTY (blok B) - od razu ---
  if (!ptyOk || ptyOut != pty) { ptyOut = pty; ptyOk = true; dirty = true; }

  if (!okA) return;                             // teksty tylko z czystym blokiem A

  switch (groupType) {
    case 0: {                                   // 0A / 0B : PS (Program Service)
      uint8_t addr = b & 0x03;                  // segment 0..3 (po 2 znaki)
      putPs(addr * 2,     d >> 8);
      putPs(addr * 2 + 1, d & 0xFF);
      break;
    }
    case 2: {                                   // 2A / 2B : RadioText
      bool    ab     = (b >> 4) & 1;            // flaga A/B - zmiana = nowy tekst
      uint8_t addr   = b & 0x0F;
      uint8_t segLen = versionB ? 2 : 4;

      if (!rtABvalid || ab != rtAB) {           // nowa wiadomość: czyść tylko bufor roboczy
        rtAB = ab; rtABvalid = true;
        memset(rtStage, ' ', 64);
        rtMask = 0; rtEnd = 64;
        rtPrevAddr = 0; rtPassMax = 0; rtPassMaxPrev = 0xFF;
      }

      // Koniec cyklu nadawania (adres zawrócił). Obsługa stacji, które nadają
      // krótszy tekst bez znaku końca 0x0D: dwa cykle z tym samym zakresem.
      if (addr < rtPrevAddr) {
        if (rtEnd >= 64 && rtPassMax < 15 && rtPassMax == rtPassMaxPrev)
          rtCommit(rtPassMax, (uint8_t)((rtPassMax + 1) * segLen));
        rtPassMaxPrev = rtPassMax;
        rtPassMax = 0;
      }
      rtPrevAddr = addr;
      if (addr > rtPassMax) rtPassMax = addr;

      uint8_t raw[4];
      if (!versionB) { raw[0] = c >> 8; raw[1] = c & 0xFF; raw[2] = d >> 8; raw[3] = d & 0xFF; }
      else           { raw[0] = d >> 8; raw[1] = d & 0xFF; }
      rtStoreSegment(addr, segLen, raw);

      if (rtEnd < 64) rtCommit(rtEnd / segLen, rtEnd);   // znamy koniec tekstu
      else            rtCommit(15, 64);                  // pełne 64 znaki
      break;
    }
    default: break;
  }
}

// Odczyt statusu radia + grupy RDS
void pollRadio() {
  if (!readRegs()) return;

  uint16_t st = reg[0];                         // 0x0A
  uint16_t q  = reg[1];                         // 0x0B

  bool    st_stereo = st & 0x0400;
  uint8_t rssi      = q >> 9;
  uint8_t bars      = (rssi > 10) ? (rssi - 10) / 8 : 0;
  if (bars > 5) bars = 5;
  if (st_stereo != stereo || bars != rssiBars) {
    stereo = st_stereo; rssiBars = bars; dirty = true;
  }

  if (!(st & 0x8000)) return;                   // RDSR: brak nowej grupy

  uint16_t a = reg[2], b = reg[3], c = reg[4], d = reg[5];
  uint8_t blerA = (q >> 2) & 3;
  uint8_t blerB = q & 3;
  if (blerB > RDS_MAX_BLER) return;             // blok B uszkodzony - odrzuć grupę

  if (a == lastA && b == lastB && c == lastC && d == lastD) return;   // duplikat
  lastA = a; lastB = b; lastC = c; lastD = d;
  lastGroupMs = millis();
  decodeGroup(a, b, c, d, blerA);
}

// ============================= STEROWANIE =============================
void settingsChanged() { settingsPending = true; settingsMs = millis(); }

void saveSettings() {
  EEPROM.update(0, 0xA7);
  EEPROM.put(1, freq100);
  EEPROM.update(3, volume);
  settingsPending = false;
}

void loadSettings() {
  if (EEPROM.read(0) == 0xA7) {
    uint16_t f; EEPROM.get(1, f);
    uint8_t v = EEPROM.read(3);
    if (f >= FREQ_MIN && f <= FREQ_MAX) freq100 = f;
    if (v <= 15) volume = v;
  }
}

// Rejestr 0x02 zależny od głośności: przy głośności 0 wyciszenie sprzętowe
// (DMUTE), bo samo VOLUME=0 w RDA5807M nie daje całkowitej ciszy.
uint16_t ctrlNow() {
  return (volume == 0) ? CTRL_MUTED : CTRL_BASE;
}

void tuneTo(uint16_t f100) {
  if (f100 < FREQ_MIN) f100 = FREQ_MIN;
  if (f100 > FREQ_MAX) f100 = FREQ_MAX;
  freq100 = f100;
  rdaWrite(REG_CONTROL, CTRL_MUTED);
  // CHAN = (f - 87,0 MHz) / 0,05 MHz ; BAND=00 (87-108), SPACE=10 (50 kHz), TUNE=1
  rdaWrite(REG_CHANNEL, (uint16_t)((((f100 - 8700) / 5) << 6) | 0x0012));
  delay(40);
  waitStc(1000);
  rdaWrite(REG_CONTROL, ctrlNow());
  resetRds();
  settingsChanged();
}

void stepFreq(int8_t dir) {
  int16_t f = (int16_t)freq100 + dir * FREQ_STEP;
  if (f > FREQ_MAX) f = FREQ_MIN;
  if (f < FREQ_MIN) f = FREQ_MAX;
  tuneTo((uint16_t)f);
}

void showMessage(const char *l1, const char *l2);

void seekStation(bool up) {
  showMessage("Szukam stacji...", up ? "w gore" : "w dol");
  // Szukanie jak dotychczas na siatce 100 kHz: dostrój na pełne 100 kHz (SPACE=00)
  rdaWrite(REG_CONTROL, CTRL_MUTED);
  rdaWrite(REG_CHANNEL, (uint16_t)(((freq100 / 10 - 870) << 6) | 0x0010));
  delay(40);
  waitStc(1000);
  rdaWrite(REG_CONTROL, CTRL_MUTED | CTRL_SEEK | (up ? CTRL_SEEKUP : 0));
  delay(60);
  bool ok = waitStc(6000);
  uint16_t ch = reg[0] & 0x03FF;               // READCHAN
  rdaWrite(REG_CONTROL, ctrlNow());
  uint16_t found = (870 + ch) * 10;            // 100 kHz -> jednostki 10 kHz
  if (ok && found >= FREQ_MIN && found <= FREQ_MAX) {
    freq100 = found;
    resetRds();
    settingsChanged();
  } else {
    tuneTo(freq100);                            // nie znaleziono - wróć na starą
  }
}

void setVolume(int8_t v) {
  if (v < 0)  v = 0;
  if (v > 15) v = 15;
  volume = v;
  rdaWrite(REG_VOLUME, 0x8880 | volume);
  rdaWrite(REG_CONTROL, ctrlNow());            // głośność 0 = całkowita cisza
  dirty = true;
  settingsChanged();
}

void rdaInit() {
  rdaWrite(REG_CONTROL, 0x0002);               // SOFT_RESET
  delay(100);
  rdaWrite(REG_CONTROL, ctrlNow());            // ENABLE, RDS_EN, NEW_METHOD
  delay(100);
  rdaWrite(REG_SYSCFG, 0x0C00);                // DE = 50 us (Europa)
  rdaWrite(REG_VOLUME, 0x8880 | volume);
  tuneTo(freq100);
}

// ============================== PRZYCISKI =============================
uint8_t btnEvent(Btn &b) {
  bool pressed = (digitalRead(b.pin) == LOW);
  uint32_t now = millis();
  if (pressed && !b.down) { b.down = true; b.longSent = false; b.t0 = now; return EV_NONE; }
  if (pressed && b.down) {
    if (!b.longSent && now - b.t0 >= LONG_PRESS_MS) { b.longSent = true; return EV_LONG; }
    return EV_NONE;
  }
  if (!pressed && b.down) {
    b.down = false;
    if (!b.longSent && now - b.t0 >= 30) return EV_SHORT;
  }
  return EV_NONE;
}

// =============================== EKRAN ================================
void getPtyName(uint8_t pty, char *out, uint8_t maxLen) {
  const char *p = PTY_NAMES;
  uint8_t idx = 0;
  while (idx < pty) {
    char c = pgm_read_byte(p++);
    if (c == '|') idx++;
    else if (c == 0) break;
  }
  uint8_t n = 0;
  while (n < maxLen - 1) {
    char c = pgm_read_byte(p++);
    if (c == '|' || c == 0) break;
    out[n++] = c;
  }
  out[n] = 0;
}

// Dzieli RadioText na RT_LINES linii: najpierw łamie na spacjach,
// a jeśli się nie mieści - łamie "twardo" co RT_LINE_CHARS znaków.
void wrapRt(const char *rt, uint8_t n) {
  for (uint8_t i = 0; i < RT_LINES; i++) rtL[i][0] = 0;
  if (n == 0) return;

  uint8_t pos = 0;
  for (uint8_t ln = 0; ln < RT_LINES && pos < n; ln++) {
    uint8_t remain = n - pos;
    uint8_t len;
    if (remain <= RT_LINE_CHARS) {
      len = remain;
    } else {
      len = RT_LINE_CHARS;                                  // brak spacji -> łamanie twarde
      for (uint8_t i = RT_LINE_CHARS; i > 0; i--)
        if (rt[pos + i] == ' ') { len = i; break; }
    }
    memcpy(rtL[ln], rt + pos, len);
    rtL[ln][len] = 0;
    pos += len;
    while (pos < n && rt[pos] == ' ') pos++;                // pomiń spacje na łamaniu
  }

  if (pos < n) {                                            // nie zmieściło się -> łamanie twarde
    for (uint8_t ln = 0; ln < RT_LINES; ln++) {
      uint8_t start = ln * RT_LINE_CHARS;
      if (start >= n) { rtL[ln][0] = 0; continue; }
      uint8_t len = n - start;
      if (len > RT_LINE_CHARS) len = RT_LINE_CHARS;
      memcpy(rtL[ln], rt + start, len);
      rtL[ln][len] = 0;
    }
  }
}

// Kopia danych do narysowania (raz na klatkę). Dzięki temu można odbierać RDS
// także w trakcie rysowania, bez ryzyka zmiany tekstu w połowie klatki.
void prepareStrings() {
  snprintf_P(sFreq, sizeof(sFreq), PSTR("%u.%02u"), (unsigned)(freq100 / 100), (unsigned)(freq100 % 100));
  snprintf_P(sVol,  sizeof(sVol),  PSTR("V:%u"), (unsigned)volume);
  memcpy(sPs, psTxt, 9);

  if (piOk) snprintf_P(sPi, sizeof(sPi), PSTR("PI:%04X"), (unsigned)piOut);
  else      strcpy_P(sPi, PSTR("PI:----"));

  if (ptyOk) {
    char name[16];
    getPtyName(ptyOut, name, sizeof(name));
    snprintf_P(sPty, sizeof(sPty), PSTR("PTY:%02u-%s"), (unsigned)ptyOut, name);
  } else {
    strcpy_P(sPty, PSTR("PTY--:---"));
  }

  dStereo = stereo;
  dRds    = rdsActive;
  dBars   = rssiBars;

  // RadioText: do pierwszego 0x0D, bez spacji na końcu
  char rt[65];
  uint8_t n = 0;
  for (uint8_t i = 0; i < 64; i++) {
    if (rtTxt[i] == '\r') break;
    rt[n++] = rtTxt[i];
  }
  while (n && rt[n - 1] == ' ') n--;
  rt[n] = 0;
  wrapRt(rt, n);

  rtLineCount = 0;                                          // ile linii ma RT
  for (uint8_t i = 0; i < RT_LINES; i++) if (rtL[i][0]) rtLineCount = i + 1;
  if (rtPage >= rtLineCount) rtPage = 0;
}

// Własne znaki LCD 1..6 = wskaźnik siły sygnału (0..5 słupków, "schodki" 1 px).
// Znak 0 nie jest używany (0 kończyłoby napis).
void initSignalGlyphs() {
  static const uint8_t H[5] = { 2, 3, 5, 6, 8 };   // wysokości słupków w pikselach
  uint8_t g[8];
  for (uint8_t lvl = 0; lvl <= 5; lvl++) {
    for (uint8_t r = 0; r < 8; r++) {                // r = 0 -> górny wiersz znaku
      uint8_t bits = 0;
      for (uint8_t i = 0; i < 5; i++) {
        uint8_t h = (i < lvl) ? H[i] : 1;            // nieaktywny słupek = kreska u dołu
        if ((7 - r) < h) bits |= (uint8_t)(0x10 >> i);
      }
      g[r] = bits;
    }
    lcd.createChar(1 + lvl, g);
  }
}

// Pomocnicze: składanie wiersza w buforze (dokładnie LCD_WIDTH znaków)
void clearLine(char *line) {
  memset(line, ' ', LCD_WIDTH);
  line[LCD_WIDTH] = 0;
}

void putAt(char *line, uint8_t col, const char *s) {
  while (*s && col < LCD_WIDTH) line[col++] = *s++;
}

void lcdRow(uint8_t row, const char *line) {
  lcd.setCursor(0, row);
  lcd.print(line);
}

// Rysuje cały ekran, wiersz po wierszu. Po każdym wierszu odbiera RDS, bo
// zapis na LCD przez I2C trwa dość długo - żadna grupa nie ginie.
void drawScreen() {
  char line[LCD_WIDTH + 1];

  // --- wiersz 1: częstotliwość, głośność, ST, siła sygnału ---
  clearLine(line);
  putAt(line, 0, sFreq);
  putAt(line, strlen(sFreq), "MHz");
  putAt(line, 10, sVol);
  if (dStereo) putAt(line, 15, "ST");
  line[19] = (char)(1 + dBars);                  // znak własny: 0..5 słupków
  lcdRow(0, line);
  pollRadio();

  // --- wiersz 2: PS, PI oraz RDS ---
  clearLine(line);
  putAt(line, 0, sPs);
  putAt(line, 9, sPi);
  if (dRds) putAt(line, 17, "RDS");
  lcdRow(1, line);
  pollRadio();

  // --- wiersz 3: PTY ---
  clearLine(line);
  putAt(line, 0, sPty);
  lcdRow(2, line);
  pollRadio();

  // --- wiersz 4: RadioText (jedna linia, zmieniana co RT_PAGE_MS) ---
  clearLine(line);
  if (rtLineCount) putAt(line, 0, rtL[(rtPage < rtLineCount) ? rtPage : 0]);
  lcdRow(3, line);
}

void drawFrame() {
  drawScreen();
  pollRadio();
}

void showMessage(const char *l1, const char *l2) {
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print(l1);
  if (l2) {
    lcd.setCursor(0, 2);
    lcd.print(l2);
  }
}

// ============================ SETUP / LOOP ============================
void setup() {
  pinMode(PIN_FREQ_UP,   INPUT_PULLUP);
  pinMode(PIN_FREQ_DOWN, INPUT_PULLUP);
  pinMode(PIN_VOL_UP,    INPUT_PULLUP);
  pinMode(PIN_VOL_DOWN,  INPUT_PULLUP);

  lcd.init();                   // inicjalizuje też Wire
  Wire.setClock(100000);
  lcd.backlight();
  initSignalGlyphs();
#if DISABLE_INTERNAL_PULLUPS
  digitalWrite(SDA, LOW);
  digitalWrite(SCL, LOW);
#endif

  loadSettings();
  resetRds();

  Wire.beginTransmission((uint8_t)RDA_ADDR_SEQ);
  if (Wire.endTransmission() != 0) {
    showMessage("BLAD I2C", "Brak RDA5807M (0x10)");
    while (true) { }
  }

  showMessage("FM Radio RDS", "RDA5807M start...");
  delay(2000);
  rdaInit();
  dirty = true;
}

void loop() {
  uint8_t e;

  e = btnEvent(btnUp);
  if (e == EV_SHORT) stepFreq(+1); else if (e == EV_LONG) seekStation(true);

  e = btnEvent(btnDown);
  if (e == EV_SHORT) stepFreq(-1); else if (e == EV_LONG) seekStation(false);

  if (btnEvent(btnVUp)) setVolume((int8_t)volume + 1);
  if (btnEvent(btnVDn)) setVolume((int8_t)volume - 1);

  static uint32_t tPoll = 0;
  if (millis() - tPoll >= 10) {
    tPoll = millis();
    pollRadio();

    bool act = (lastGroupMs != 0) && (millis() - lastGroupMs < 5000);
    if (act != rdsActive) { rdsActive = act; dirty = true; }
  }

  if (millis() - rtPageMs >= RT_PAGE_MS) {      // kolejna linia RadioText
    rtPageMs = millis();
    if (rtLineCount > 1) { rtPage = (rtPage + 1) % rtLineCount; dirty = true; }
  }

  static uint32_t tDraw = 0;
  if (dirty && millis() - tDraw >= 100) {
    dirty = false;              // zmiany w trakcie rysowania ustawią flagę ponownie
    prepareStrings();
    drawFrame();
    tDraw = millis();
  }

  if (settingsPending && millis() - settingsMs > 3000) saveSettings();
}
