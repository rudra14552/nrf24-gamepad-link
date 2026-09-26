/*
  ESP32-S3 -> USB gamepad dongle -> Ebyte E01-2G4M27D (nRF24 PA/LNA)
  =================================================================
  TRANSMITTER  -  EspUsbHost v2  +  event-driven low-latency radio
                  +  Layer-2 control panel (OLED + rotary encoder + 4-button d-pad)

  ======================================================================
  TRANSMIT PATTERN - READ THIS FIRST
  ======================================================================
  Old design: send every 10 ms on a fixed clock, always. That put up to
  10 ms of pure *waiting* latency between "you move the stick" and
  "a packet leaves the antenna", AND burned airtime sending identical
  data when nothing moved. Bad for both latency and bandwidth.

  New design: SEND-ON-CHANGE, rate-capped, with a keepalive heartbeat.

    1. A USB HID report arrives -> decoded into `rawLX/rawLY/rawRX/rawRY`
       plus buttons/dpad/triggers directly into `state`.
    2. Every loop, trim/reverse/expo (or, for the throttle axis, the
       default/near-linear curve - see THROTTLE / ALTITUDE below) are
       applied to the raw stick bytes to produce the values that
       actually go out - so a trim change made on the OLED menu shows
       up in the very next packet, even if the gamepad itself hasn't
       moved.
    3. If the resulting `state` differs from the last packet we actually
       sent (compared byte-for-byte, excluding `seq`), and at least
       MIN_TX_INTERVAL_MS has passed since the last send -> send NOW.
       This is the latency-critical path: worst case latency is one
       USB polling interval + MIN_TX_INTERVAL_MS, not a fixed 10 ms.
    4. If nothing changed, we still send once every HEARTBEAT_MS so the
       receiver can tell "link idle" from "link dead" and so the radio
       proves itself alive with no gamepad attached.
    5. MIN_TX_INTERVAL_MS exists ONLY to cap worst-case airtime/duty
       cycle if the dongle reports very fast (some pads hit 1000 Hz) -
       it is deliberately small (see the math below) so it barely
       touches latency at all.

  AIRTIME MATH (why MIN_TX_INTERVAL_MS=4 is safe for the E01)
  -------------------------------------------------------------
  Packet on air = preamble+address+payload+CRC =~ 1(preamble)+5(addr)+
  11(payload)+2(CRC16) = 19 bytes =~ 152 bits. At 1 Mbps that is
  ~152 us of actual air time. Sending every 4 ms is a ~3.8% duty
  cycle - trivial for the module, and it means change-triggered
  packets are delayed by at most 4 ms even in the worst case of the
  gamepad spamming changes faster than that.

  BANDWIDTH: with send-on-change, a controller sitting idle transmits
  only ~20 packets/sec (the heartbeat) instead of 100-1000/sec. Moving
  a stick sends packets close to the gamepad's own report rate, capped
  at 250 Hz by MIN_TX_INTERVAL_MS. Either way, the receiver decodes the
  exact same fixed 11-byte struct - no framing, no varint, nothing to
  parse. Simple format, low bandwidth, low latency: the fixed struct
  gives you "easy decode", and send-on-change gives you the other two.

  ======================================================================
  THROTTLE / ALTITUDE AXIS - dual curve, button-selected
  ======================================================================
  Auto-centering sticks make a normal drone throttle (which needs to
  HOLD a position) awkward, so the throttle axis gets its own mapping
  instead of the shared trim/expo path the other three axes use:

    - DEFAULT (button released) -> smooth non-linear expo curve ->
      gentle near center for fine altitude control, stronger response
      near the extremes. How strong it is can be changed on the
      THROTTLE CURVE menu page (saved to flash).
    - AGGRESSIVE_BTN_MASK held (ML paddle by default) -> steep,
      near-linear curve -> instant, snappy climb/dive.

  This is a pure function of CURRENT stick position + CURRENT button
  state, re-evaluated fresh every loop tick - no ramping, no memory of
  the last position, no special-casing the moment the button is
  pressed or released. Whatever quadrant the stick happens to be
  sitting in when the button changes state just gets pushed through
  whichever curve applies right now.

  THROTTLE_IS_LY (below) assumes the LEFT stick's vertical axis is your
  throttle/altitude channel (Mode-2 style). Flip it to 0 if your
  controller uses RY instead. AGGRESSIVE_BTN_MASK defaults to ML
  (byte[1] bit 0x04 -> 0x0400 in the packet's `buttons` field) - change
  it to whatever button you want to hold for the snappy response.

  ======================================================================
  LAYER-2 CONTROL PANEL (OLED + rotary encoder + 4-button d-pad)
  ======================================================================
  Adds on-the-fly TRIM (per-stick X/Y offset), a live telemetry/status view on the default screen,
  per-axis REVERSE, an adjustable THROTTLE CURVE (drawn as a live graph),
  and on-screen live controller/Tx status, as three menu PAGES you
  step through with the OLED - no laptop, no phone, nothing but the
  transmitter itself. Settings are saved to flash (Preferences/NVS) so
  they survive a power cycle.

  IMPORTANT - the radio packet is UNCHANGED. Still 11 bytes, same
  layout as before. Trim/expo/reverse/throttle-curve are applied on
  THIS board, before the bytes are packed, so your existing receiver
  needs zero changes. The one new thing on the wire is that bit 1 of


  MENU STRUCTURE - one page at a time, all OLED text centered:

    STATUS screen (default):
      press encoder      -> open menu (starts on the TRIM page)
      long-press encoder  -> quick ARM/DISARM toggle, deliberate hold
                             so it can't fire from a stray tap

    Inside the menu, at the top of ANY page (not drilled into an item):
      rotate encoder -> select the item highlighted on this page
                        (rotation NEVER changes the page - it only
                        moves the highlight, wrapping at the ends)
      LEFT / RIGHT   -> switch to the previous/next page (cyclic)
      long-press enc -> save + exit all the way back to STATUS

    PAGE 1 - TRIM
      browsing: rotate encoder to choose "Left Stick" or "Right Stick",
                press encoder to drill into that stick's trim
      editing:  UP/DOWN nudge that stick's Y trim, LEFT/RIGHT nudge its
                X trim - all 4 push buttons, direct control
                press encoder -> back to stick selection (this page's
                "previous page")

    PAGE 2 - LIVE DATA
      view-only: shows LX/LY/RX/RY (post trim/expo/throttle-curve),
      LT/RT, the raw buttons bitmask and dpad value, refreshed live

    PAGE 3 - CHANNEL INVERT
      browsing: rotate encoder to choose LX/LY/RX/RY, press encoder to
      toggle that channel's reverse flag

    PAGE 4 - THROTTLE CURVE
      Adjusts the DEFAULT (non-linear) throttle curve - the one used
      while the aggressive/ML button is NOT held. Shows the curve as a
      graph (solid line) against the straight linear reference (dotted
      diagonal), plus the set value as a small number (0-100 %).
        rotate encoder -> change the value in steps of CURVE_STEP_COARSE
                          (rotation still never changes the page)
        UP / DOWN      -> fine adjust in steps of CURVE_STEP_FINE
        LEFT / RIGHT   -> change page (the value is saved on the way out)
      0 % = straight linear, 100 % = full cubic (very soft near center).

    PAGE 5 - ARM / RESET
      browsing: rotate encoder to choose "ARM/DISARM" or "Reset trims
      to 0", press encoder to execute the highlighted action

    10 seconds with no input while inside the menu auto-saves and
    returns to the STATUS screen (except on the LIVE DATA page).

  Wiring (control panel) - none of this conflicts with the nRF24 SPI
  pins (4,5,6,7,15) or the native-USB host port (19/20, fixed). GPIO 0,
  45 (boot-strapping pins) and 19 (USB D-) are intentionally avoided:
    OLED SDA   -> GPIO 8
    OLED SCL   -> GPIO 9
    Encoder CLK (A)  -> GPIO 42
    Encoder DT  (B)  -> GPIO 2
    Encoder SW       -> GPIO 1
    D-pad UP    -> GPIO 40
    D-pad DOWN  -> GPIO 39
    D-pad LEFT  -> GPIO 41
    D-pad RIGHT -> GPIO 38
  All buttons/encoder-switch wired to GND on the other leg (internal
  pull-ups are enabled in software). Change the #defines below if your
  wiring differs. If the OLED stays blank, try address 0x3D instead of
  0x3C. If the encoder counts backwards, swap the A/B wires.

  Libraries needed (Library Manager): "Adafruit SSD1306", "Adafruit GFX
  Library". Preferences.h is part of the ESP32 Arduino core already.

  ======================================================================
  HOW TO DISABLE SERIAL PRINTING TO SAVE CPU CYCLES
  ======================================================================
  Serial printing is OFF by default now. To turn debug output back ON
  (useful while testing the control panel/trims), change ONE line
  below:

        #define DEBUG_PRINT      0        ->      #define DEBUG_PRINT      1

  Every Serial.print/printf in this file is wrapped in "#if DEBUG_PRINT"
  blocks, so leaving it at 0 removes them all from the compiled binary
  (not just skips them at runtime - the compiler deletes the code
  entirely, so it costs zero flash and zero cycles). The line to change
  is in the USER SETTINGS block just below.

  Wiring (radio + gamepad)
  ------
    E01 VCC  -> 3V3   *** SEE POWER NOTE BELOW ***
    E01 GND  -> GND
    E01 CE   -> GPIO 4
    E01 CSN  -> GPIO 5
    E01 SCK  -> GPIO 6
    E01 MOSI -> GPIO 7
    E01 MISO -> GPIO 15
    E01 IRQ  -> GPIO 16  (wired, unused)
    Gamepad dongle -> native USB host port (GPIO19/20, fixed)
    Serial monitor -> the board's UART/USB-bridge port

  POWER NOTE
  ----------
  The E01-2G4M27D draws roughly 500-650 mA in TX bursts at full power.
  Feed it from a separate 3.3V / 1A+ regulator, common ground with the
  ESP32, with 220-470 uF electrolytic + 100 nF ceramic right at the
  module's VCC/GND pins.

  Libraries: RF24 by TMRh20, EspUsbHost by tanakamasayuki (v2.x),
             Adafruit SSD1306, Adafruit GFX Library
*/

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include "EspUsbHost.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
#include <esp_system.h>
#include <math.h>
#include <string.h>

// =====================================================================
// USER SETTINGS
// =====================================================================
#define DEBUG_PRINT      0     // <-- SET TO 1 TO ENABLE SERIAL OUTPUT (uses more CPU)
#define DEBUG_RAW_BYTES  0     // 1 = dump raw HID bytes instead of decoded values
#define DEBUG_HZ         5     // status print rate when DEBUG_PRINT=1 (does not affect TX)

#define NRF_CHANNEL       76   // 2476 MHz - in band, in the E01's passband
#define NRF_PA_LEVEL      RF24_PA_MAX  // bench test LOW, raise once link is proven
#define NRF_DATARATE      RF24_1MBPS

#define MIN_TX_INTERVAL_MS  4   // cap on send rate when data is changing (~250 Hz max)
#define HEARTBEAT_MS        50  // send even with no change at least this often

// nRF24 pins
#define PIN_CE    4
#define PIN_CSN   5
#define PIN_SCK   6
#define PIN_MOSI  7
#define PIN_MISO 15

// ---------------------------------------------------------------------
// Layer-2 control panel pins - change to match your wiring.
// ---------------------------------------------------------------------
#define OLED_SDA      8
#define OLED_SCL      9
#define OLED_ADDR     0x3C     // some 0.96" modules are 0x3D - change if screen stays blank
#define OLED_WIDTH    128
#define OLED_HEIGHT   64

#define ENC_A         42     // CLK
#define ENC_B         2      // DT
#define ENC_BTN       1      // SW

#define BTN_UP        40
#define BTN_DOWN      39
#define BTN_LEFT      41
#define BTN_RIGHT     38

#define TRIM_STEP            1        // raw units per button press
#define TRIM_MAX             60       // max +/- trim, raw 0-255 units (~24% of travel)
#define DEBOUNCE_MS          25
#define LONGPRESS_MS         700
#define MENU_IDLE_TIMEOUT_MS 10000UL  // auto-save & return to status screen
#define OLED_REFRESH_HZ      12       // keep low - I2C writes are comparatively slow
#define DEFAULT_THROTTLE_EXPO 65      // factory value of the throttle's default (non-linear) curve, 0-100 %
#define CURVE_STEP_COARSE     5       // expo change per encoder detent on the THROTTLE CURVE page
#define CURVE_STEP_FINE       1       // expo change per UP/DOWN press on the THROTTLE CURVE page

// =====================================================================
// REPORT LAYOUT - CONFIRMED MAPPING for your EvoFox One S, taken from
// your verified gamepad_serial_premapped.ino. This replaces the
// earlier placeholder offsets, which were wrong (they assumed one
// contiguous 16-bit button field starting at byte 1 - the real device
// splits buttons across two separate bit-flag bytes at 0 and 1, with
// the dpad and sticks shifted down accordingly).
//
//   Digital buttons - byte[0] bit flags:
//     0x01 X   0x02 A   0x04 B   0x08 Y
//     0x10 LB  0x20 RB  0x40 LT(click)  0x80 RT(click)
//   Digital buttons - byte[1] bit flags:
//     0x01 SELECT  0x02 START  0x04 ML(paddle)  0x08 MR(paddle)  0x10 HOME
//
//   D-pad (hat) - byte[2]:
//     0x08 = neutral, 0x00 = up, 0x02 = right, 0x04 = down, 0x06 = left
//     (odd values are diagonals if your unit reports them - unconfirmed)
//
//   Sticks (0x00-0xFF, 0x80 = center) - byte[3..6]:
//     LX=3 (0=left,255=right)  LY=4 (0=up,255=down)
//     RX=5 (0=left,255=right)  RY=6 (0=up,255=down)
//
//   Analog trigger pressure (0x00-0xFF, rises with pull) - byte[17],[18]:
//     LT_ANALOG=17  RT_ANALOG=18
//   (bytes 7-16 also carry per-button analog pressure on this dongle -
//    right/left/up/down/Y/B/A/X/LB/RB in that order - not used here
//    since we only have 2 spare bytes in the radio packet for
//    "analog extras"; LT/RT were picked as the most useful pair)
// =====================================================================
struct ReportLayout {
  uint8_t btnLowByte  = 0;    // X,A,B,Y,LB,RB,LT-click,RT-click
  uint8_t btnHighByte = 1;    // SELECT,START,ML,MR,HOME (only 5 bits used)
  uint8_t dpadByte    = 2;
  uint8_t lxByte      = 3;
  uint8_t lyByte      = 4;
  uint8_t rxByte      = 5;
  uint8_t ryByte      = 6;
  uint8_t ltByte      = 17;   // analog trigger pressure, not the click bit
  uint8_t rtByte      = 18;   // analog trigger pressure, not the click bit
} layout;

// =====================================================================
// Radio payload - packed, fixed 11 bytes. MUST match the receiver
// byte-for-byte. This fixed layout is what makes decoding trivial:
// no delimiters, no length prefix, no varint - just cast the bytes.
// =====================================================================
#pragma pack(push, 1)
struct GamepadPacket {
  uint8_t  seq;       // increments on every packet actually sent
  uint8_t  flags;     // bit0: 1 = live gamepad data, 0 = heartbeat/default
                       // bit1: 1 = ARMED, 0 = safe (see LAYER-2 CONTROL PANEL above)
  uint16_t buttons;   // low byte = HID byte[0], high byte = HID byte[1]:
                       //   0x0001 X     0x0002 A     0x0004 B     0x0008 Y
                       //   0x0010 LB    0x0020 RB    0x0040 LT-click
                       //   0x0080 RT-click
                       //   0x0100 SELECT 0x0200 START 0x0400 ML(paddle)
                       //   0x0800 MR(paddle) 0x1000 HOME
  uint8_t  dpad;      // raw HID byte[2]: 8=neutral,0=up,2=right,4=down,6=left
  uint8_t  lx, ly;    // 0-255, 128=center - AFTER trim/expo/reverse applied
  uint8_t  rx, ry;    // 0-255, 128=center - AFTER trim/expo/reverse applied
  uint8_t  lt, rt;    // ANALOG trigger pressure (0-255), not the click bit -
                       // the click bit lives in `buttons` (0x0040 / 0x0080)
};
#pragma pack(pop)
static_assert(sizeof(GamepadPacket) == 11, "payload must stay 11 bytes");

// Explicit prototype - same reasoning as pollButton's, above: GamepadPacket
// is a custom type, so we declare this ourselves to stop the IDE inserting
// a broken auto-generated prototype before the struct is defined.
static bool payloadChanged(const GamepadPacket &a, const GamepadPacket &b);

#define FLAG_GAMEPAD_LIVE  0x01

// Bits inside cfg.revMask
#define REV_LX_BIT  0x01
#define REV_LY_BIT  0x02
#define REV_RX_BIT  0x04
#define REV_RY_BIT  0x08

// =====================================================================
// Shared state: EspUsbHost's background task writes here via the
// onHIDInput callback; loop() reads it. Spinlock avoids a torn read.
// =====================================================================
static portMUX_TYPE reportMux = portMUX_INITIALIZER_UNLOCKED;
static uint8_t  rawReport[64];
static uint8_t  rawReportLen = 0;
static volatile bool newReportFlag = false;

EspUsbHost usb;
RF24 radio(PIN_CE, PIN_CSN);
SPIClass radioSpi(FSPI);

const uint8_t txAddress[6] = "CTRL1";

static GamepadPacket state;       // latest values, sent as-is over the radio
static GamepadPacket lastSent;    // the payload of the last packet we transmitted
static bool     haveLastSent = false;
static uint32_t lastTxMs     = 0;
static uint32_t lastPrintMs  = 0;
static uint32_t lastReportMs = 0;
static uint32_t txCount      = 0;

static uint8_t  dbgReport[64];
static uint8_t  dbgLen = 0;

// Raw stick bytes straight from the HID report, BEFORE trim/expo/reverse.
// state.lx/ly/rx/ry are recomputed from these every loop.
static uint8_t rawLX = 128, rawLY = 128, rawRX = 128, rawRY = 128;

// Compare two packets ignoring `seq` (seq always differs by design,
// so it must not count as a "change" for send-on-change purposes).
static bool payloadChanged(const GamepadPacket &a, const GamepadPacket &b) {
  return memcmp(reinterpret_cast<const uint8_t*>(&a) + 1,
                reinterpret_cast<const uint8_t*>(&b) + 1,
                sizeof(GamepadPacket) - 1) != 0;
}

// =====================================================================
// LAYER-2 CONTROL PANEL: trim/expo/reverse config, persisted to flash.
// =====================================================================
struct TrimConfig {
  int16_t trimLX = 0;
  int16_t trimLY = 0;
  int16_t trimRX = 0;
  int16_t trimRY = 0;
  uint8_t expo   = 0;     // 0-100 %  (applied to the three non-throttle axes)
  uint8_t revMask = 0;    // bit0=LX bit1=LY bit2=RX bit3=RY
  uint8_t throttleExpo = DEFAULT_THROTTLE_EXPO;   // 0-100 %, strength of the throttle's default (non-linear) curve
};
TrimConfig cfg;
Preferences prefs;

void loadConfig() {
  prefs.begin("trimcfg", true);   // read-only open
  cfg.trimLX  = prefs.getShort("lx", 0);
  cfg.trimLY  = prefs.getShort("ly", 0);
  cfg.trimRX  = prefs.getShort("rx", 0);
  cfg.trimRY  = prefs.getShort("ry", 0);
  cfg.expo    = prefs.getUChar("expo", 0);
  cfg.revMask = prefs.getUChar("rev", 0);
  cfg.throttleExpo = prefs.getUChar("texpo", DEFAULT_THROTTLE_EXPO);
  if (cfg.throttleExpo > 100) cfg.throttleExpo = 100;
  prefs.end();
}

void saveConfig() {
  prefs.begin("trimcfg", false);  // read-write open
  prefs.putShort("lx", cfg.trimLX);
  prefs.putShort("ly", cfg.trimLY);
  prefs.putShort("rx", cfg.trimRX);
  prefs.putShort("ry", cfg.trimRY);
  prefs.putUChar("expo", cfg.expo);
  prefs.putUChar("rev", cfg.revMask);
  prefs.putUChar("texpo", cfg.throttleExpo);
  prefs.end();
}

// Apply reverse -> trim -> expo to one raw axis byte (0-255, 128=center).
// Used for the three non-throttle axes.
uint8_t applyAxis(uint8_t raw, int16_t trim, bool reversed, uint8_t expoPct) {
  int16_t v = reversed ? (255 - raw) : raw;
  v += trim;
  if (v < 0)   v = 0;
  if (v > 255) v = 255;

  if (expoPct > 0) {
    float x = (v - 128) / 128.0f;              // -1..+1
    float e = expoPct / 100.0f;
    float y = e * x * x * x + (1.0f - e) * x;  // classic RC expo curve
    v = 128 + (int16_t)lroundf(y * 128.0f);
    if (v < 0)   v = 0;
    if (v > 255) v = 255;
  }
  return (uint8_t)v;
}

// ---------------------------------------------------------------------
// THROTTLE / ALTITUDE axis - dual curve, button-selected. See the big
// comment block near the top of the file for the full rationale.
//   default (button released) -> smooth non-linear expo curve
//   AGGRESSIVE_BTN_MASK held  -> near-linear curve (instant response)
// ---------------------------------------------------------------------
#define THROTTLE_IS_LY        1        // 1 = LY is throttle, 0 = RY is throttle
#define AGGRESSIVE_BTN_MASK   0x0400   // ML paddle (byte[1] bit 0x04) - hold for instant/near-linear response

static inline float throttleAggressiveCurve(float x) {
  return x;   // near-linear -> instant, snappy climb/dive
              // (want it very slightly softened? try: 0.15f*x*x*x + 0.85f*x)
}
// e = 0..1 (cfg.throttleExpo / 100): higher = gentler near center, more
// bite at the ends. Adjusted live on the THROTTLE CURVE menu page.
static inline float throttleNormalCurve(float x, float e) {
  return e * x * x * x + (1.0f - e) * x;
}

// Same reverse -> trim handling as applyAxis(), then picks whichever
// curve the aggressive button currently selects. Pure function of the
// CURRENT raw position + CURRENT button state - no history kept.
uint8_t applyThrottle(uint8_t raw, int16_t trim, bool reversed, bool aggressive) {
  int16_t v = reversed ? (255 - raw) : raw;
  v += trim;
  if (v < 0)   v = 0;
  if (v > 255) v = 255;

  float x = (v - 128) / 128.0f;   // current position, -1..+1
  float y = aggressive ? throttleAggressiveCurve(x)
                       : throttleNormalCurve(x, cfg.throttleExpo / 100.0f);
  int16_t out = 128 + (int16_t)lroundf(y * 128.0f);
  if (out < 0)   out = 0;
  if (out > 255) out = 255;
  return (uint8_t)out;
}

// ---------------------------------------------------------------------
// Rotary encoder - proper 4x quadrature decode via lookup table, fired
// on CHANGE from BOTH encoder pins. This replaces an earlier "falling
// edge of A, read B" approach that could skip or double-count steps on
// some encoders. Each mechanical detent produces 4 quarter-step
// transitions; we accumulate them and only register a tick once a full
// detent's worth (4) has been seen in one consistent direction, which
// also acts as the debounce.
// ---------------------------------------------------------------------
volatile int32_t encDelta   = 0;   // accumulated full ticks, consumed by handleControls()
volatile uint8_t lastEncState = 0; // last 2-bit (CLK<<1 | DT) state

void IRAM_ATTR encoderISR() {
  uint8_t clk = digitalRead(ENC_A);
  uint8_t dt  = digitalRead(ENC_B);
  uint8_t encNow = (clk << 1) | dt;

  static const int8_t table[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
  };

  uint8_t index = (lastEncState << 2) | encNow;
  int8_t movement = table[index & 0x0F];

  static int8_t accum = 0;
  accum += movement;

  if (accum >= 4) {
    encDelta++;
    accum = 0;
  } else if (accum <= -4) {
    encDelta--;
    accum = 0;
  }
  lastEncState = encNow;
}

// ---------------------------------------------------------------------
// Debounced buttons with short-press (fires on release) and long-press
// (fires once, while still held, after LONGPRESS_MS).
// ---------------------------------------------------------------------
enum BtnEvent { BTN_NONE, BTN_SHORT, BTN_LONG };

struct Btn {
  uint8_t  pin;
  bool     stable       = HIGH;   // pulled up -> HIGH = not pressed
  bool     lastRead      = HIGH;
  uint32_t lastChangeMs  = 0;
  uint32_t pressStartMs  = 0;
  bool     longFired     = false;
};

Btn btnUp{BTN_UP}, btnDown{BTN_DOWN}, btnLeft{BTN_LEFT}, btnRight{BTN_RIGHT}, btnEnc{ENC_BTN};

// Explicit prototype - prevents the Arduino IDE's auto-generated prototype
// (which it inserts near the TOP of the file, before BtnEvent/Btn exist)
// from breaking the build with "does not name a type".
BtnEvent pollButton(Btn &b);


BtnEvent pollButton(Btn &b) {
  bool raw = digitalRead(b.pin);
  uint32_t now = millis();

  if (raw != b.lastRead) {
    b.lastRead = raw;
    b.lastChangeMs = now;
  }

  if ((now - b.lastChangeMs) > DEBOUNCE_MS && raw != b.stable) {
    b.stable = raw;
    if (b.stable == LOW) {
      b.pressStartMs = now;      // just pressed
      b.longFired = false;
    } else if (!b.longFired) {   // just released, and wasn't already a long-press
      return BTN_SHORT;
    }
  }

  if (b.stable == LOW && !b.longFired && (now - b.pressStartMs) > LONGPRESS_MS) {
    b.longFired = true;
    return BTN_LONG;
  }
  return BTN_NONE;
}

// ---------------------------------------------------------------------
// Menu state machine - one page at a time. See the MENU STRUCTURE
// comment block near the top of the file for the full control scheme.
// ---------------------------------------------------------------------
enum Screen   { SCR_STATUS, SCR_MENU };
enum Page     { PAGE_TRIM, PAGE_INVERT, PAGE_CURVE, PAGE_COUNT };
enum SubState { SUB_BROWSE, SUB_EDIT };   // SUB_EDIT only used on PAGE_TRIM

Screen   screen      = SCR_STATUS;
Page     currentPage = PAGE_TRIM;
SubState subState    = SUB_BROWSE;

int8_t   trimSel       = 0;   // 0 = Left Stick, 1 = Right Stick   (PAGE_TRIM)
int8_t   invertSel     = 0;   // 0..3 = LX,LY,RX,RY                (PAGE_INVERT)
uint32_t lastActivityMs = 0;

Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
static bool oledPresent = false;   // only true if oled.begin() actually succeeded

// =====================================================================
// OLED runs on its own FreeRTOS task, pinned to the OTHER core from the
// radio/USB loop. Why: oled.display() pushes the whole 1KB framebuffer
// over I2C - at 400kHz that's multiple milliseconds, and that used to
// happen INLINE inside loop(), stalling gamepad-read -> radio-send by
// that same amount every ~83 ms (OLED_REFRESH_HZ). Moving it off means
// the I2C transfer time can never eat into TX latency, no matter how
// slow the display bus is.
//
// loop() never touches the OLED directly. It just copies the handful
// of values the screen needs into `dispSnap` under a spinlock (a few
// dozen bytes - microseconds, not milliseconds) and moves on. The OLED
// task wakes up on its own schedule, copies the snapshot out, and does
// all the slow I2C work independently.
// =====================================================================
struct DisplaySnapshot {
  Screen        screen;
  Page          currentPage;
  SubState      subState;
  int8_t        trimSel, invertSel;
  TrimConfig    cfg;
  uint32_t      txCount;
  GamepadPacket state;   // lx/ly/rx/ry/lt/rt/buttons/dpad/flags for the default live screen
};
static portMUX_TYPE   dispMux = portMUX_INITIALIZER_UNLOCKED;
static DisplaySnapshot dispSnap;

// Explicit prototypes - same reasoning as pollButton's/payloadChanged's,
// above: DisplaySnapshot is a custom type defined here, partway through
// the file, so the Arduino IDE's auto-generated prototypes (which it
// inserts near the TOP of the file, before DisplaySnapshot exists) would
// otherwise fail to compile. Declaring them ourselves, after the type is
// known, stops the IDE from generating its own broken versions.
void drawStatusScreen(const DisplaySnapshot &d);
void drawTrimPage(const DisplaySnapshot &d);
void drawInvertPage(const DisplaySnapshot &d);
void drawCurvePage(const DisplaySnapshot &d);
void oledTask(void *pvParameters);

void handleControls() {
  noInterrupts();
  int32_t delta = encDelta;
  encDelta = 0;
  interrupts();

  BtnEvent up    = pollButton(btnUp);
  BtnEvent down  = pollButton(btnDown);
  BtnEvent left  = pollButton(btnLeft);
  BtnEvent right = pollButton(btnRight);
  BtnEvent enc   = pollButton(btnEnc);

  if (delta != 0 || up || down || left || right || enc) {
    lastActivityMs = millis();
  }

  if (screen == SCR_STATUS) {
    if (enc == BTN_SHORT) {              // open menu, always starts on TRIM page
      screen = SCR_MENU;
      currentPage = PAGE_TRIM;
      subState = SUB_BROWSE;
    }
    return;
  }

  // screen == SCR_MENU from here down. A long-press ANYWHERE inside the
  // menu is the deliberate, can't-fire-by-accident way to save and
  // fully exit back to STATUS.
  if (enc == BTN_LONG) {
    saveConfig();
    screen = SCR_STATUS;
    subState = SUB_BROWSE;
    return;
  }

  if (subState == SUB_BROWSE) {
    // Encoder rotation ONLY moves the selection on the current page
    // (wrapping at the ends). It never changes the page - that's what
    // LEFT/RIGHT are for.
    if (delta != 0) {
      int8_t dir = (delta > 0) ? 1 : -1;
      switch (currentPage) {
        case PAGE_TRIM:   trimSel   = (int8_t)((trimSel   + dir + 2) % 2); break;
        case PAGE_INVERT: invertSel = (int8_t)((invertSel + dir + 4) % 4); break;
        case PAGE_CURVE:  // rotation edits the curve value here - still never changes page
          cfg.throttleExpo = (uint8_t)constrain((int)cfg.throttleExpo + (int)delta * CURVE_STEP_COARSE, 0, 100);
          break;
        default: break;
      }
    }

    // Page navigation - only live while browsing, so LEFT/RIGHT never
    // get stolen from PAGE_TRIM's edit mode below.
    // Fine adjust of the throttle curve on its page (UP/DOWN are otherwise
    // unused while browsing).
    if (currentPage == PAGE_CURVE) {
      if (up   == BTN_SHORT) cfg.throttleExpo = (uint8_t)constrain((int)cfg.throttleExpo + CURVE_STEP_FINE, 0, 100);
      if (down == BTN_SHORT) cfg.throttleExpo = (uint8_t)constrain((int)cfg.throttleExpo - CURVE_STEP_FINE, 0, 100);
    }

    if (left == BTN_SHORT) {
      if (currentPage == PAGE_CURVE) saveConfig();   // persist the curve on the way out
      currentPage = (Page)((currentPage + PAGE_COUNT - 1) % PAGE_COUNT);
    }
    if (right == BTN_SHORT) {
      if (currentPage == PAGE_CURVE) saveConfig();
      currentPage = (Page)((currentPage + 1) % PAGE_COUNT);
    }

    if (enc == BTN_SHORT) {
      switch (currentPage) {
        case PAGE_TRIM:
          subState = SUB_EDIT;              // drill into the selected stick
          break;
        case PAGE_INVERT:
          cfg.revMask ^= (1 << invertSel);  // toggle immediately
          saveConfig();
          break;
        case PAGE_CURVE:
          break;
        default:
          break;
      }
    }
  } else {
    // SUB_EDIT - only reachable from PAGE_TRIM (nudge trim with all 4
    // push buttons).
    if (currentPage == PAGE_TRIM) {
      int16_t *tx = (trimSel == 0) ? &cfg.trimLX : &cfg.trimRX;
      int16_t *ty = (trimSel == 0) ? &cfg.trimLY : &cfg.trimRY;

      if (up    == BTN_SHORT) *ty = (int16_t)constrain((int)*ty + TRIM_STEP, -TRIM_MAX, TRIM_MAX);
      if (down  == BTN_SHORT) *ty = (int16_t)constrain((int)*ty - TRIM_STEP, -TRIM_MAX, TRIM_MAX);
      if (right == BTN_SHORT) *tx = (int16_t)constrain((int)*tx + TRIM_STEP, -TRIM_MAX, TRIM_MAX);
      if (left  == BTN_SHORT) *tx = (int16_t)constrain((int)*tx - TRIM_STEP, -TRIM_MAX, TRIM_MAX);

      if (enc == BTN_SHORT) {
        saveConfig();
        subState = SUB_BROWSE;   // back to stick selection - this page's "previous page"
      }
    } else {
      subState = SUB_BROWSE;     // safety net: SUB_EDIT has no meaning on other pages
    }
  }

  // Bail out of the menu automatically so you're never stuck off the
  // status screen without noticing.
  if (screen == SCR_MENU &&
      (millis() - lastActivityMs) > MENU_IDLE_TIMEOUT_MS) {
    saveConfig();
    screen = SCR_STATUS;
    subState = SUB_BROWSE;
  }
}

// ---------------------------------------------------------------------
// OLED drawing - all text centered horizontally, plain-text, and now
// entirely run from oledTask() on the other core (see dispMux/dispSnap
// above) so it can never eat into the latency budget of the radio loop.
// ---------------------------------------------------------------------
void printCentered(const char *text, int16_t y, uint8_t size = 1) {
  oled.setTextSize(size);
  oled.setTextColor(SSD1306_WHITE);
  int16_t x1, y1;
  uint16_t w, h;
  oled.getTextBounds(text, 0, y, &x1, &y1, &w, &h);
  int16_t x = (int16_t)((OLED_WIDTH - (int16_t)w) / 2);
  if (x < 0) x = 0;
  oled.setCursor(x, y);
  oled.print(text);
}

// Bit order matches the GamepadPacket.buttons comment block up top:
// low byte (HID byte[0]) then high byte (HID byte[1]), LSB first.
static const char *BUTTON_NAMES[13] = {
  "X", "A", "B", "Y", "LB", "RB", "LT", "RT",
  "SEL", "STA", "ML", "MR", "HOME"
};

// Builds a space-separated list of currently-pressed button names, e.g.
// "A B LB" - into `out`. Writes "-" if nothing is pressed.
static void buttonsToNames(uint16_t buttons, char *out, size_t outSize) {
  out[0] = '\0';
  for (uint8_t i = 0; i < 13; i++) {
    if (!(buttons & (1u << i))) continue;
    if (out[0] != '\0') strncat(out, " ", outSize - strlen(out) - 1);
    strncat(out, BUTTON_NAMES[i], outSize - strlen(out) - 1);
  }
  if (out[0] == '\0') strncpy(out, "-", outSize - 1);
}

// Raw hat value -> direction name. See the ReportLayout comment up top:
// 0x08 neutral, 0x00 up, 0x02 right, 0x04 down, 0x06 left. Diagonals
// (odd values) aren't confirmed on this dongle, so they fall through to
// showing the raw hex rather than guessing a wrong name.
static const char *dpadToName(uint8_t dpad, char *hexFallback, size_t hexSize) {
  switch (dpad) {
    case 0x08: return "-";
    case 0x00: return "UP";
    case 0x02: return "RIGHT";
    case 0x04: return "DOWN";
    case 0x06: return "LEFT";
    default:
      snprintf(hexFallback, hexSize, "0x%02X", dpad);
      return hexFallback;
  }
}

void drawStatusScreen(const DisplaySnapshot &d) {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);

  char buf[32];

  // Live analog inputs: all 6 values are visible on the home screen.
  snprintf(buf, sizeof(buf), "LX:%3u LY:%3u", d.state.lx, d.state.ly);
  oled.setCursor(0, 0);  oled.print(buf);
  snprintf(buf, sizeof(buf), "RX:%3u RY:%3u", d.state.rx, d.state.ry);
  oled.setCursor(0, 10); oled.print(buf);
  snprintf(buf, sizeof(buf), "LT:%3u RT:%3u", d.state.lt, d.state.rt);
  oled.setCursor(0, 20); oled.print(buf);

  // Show real button names instead of the raw hexadecimal button mask.
  char buttonNames[32];
  buttonsToNames(d.state.buttons, buttonNames, sizeof(buttonNames));
  oled.setCursor(0, 30);
  oled.print("BTN:");
  oled.print(buttonNames);

  // D-pad is shown by its real name.
  char dpadHex[8];
  const char *dpadName = dpadToName(d.state.dpad, dpadHex, sizeof(dpadHex));
  oled.setCursor(0, 40);
  oled.print("DP:");
  oled.print(dpadName);

  // TX count + link state.
  snprintf(buf, sizeof(buf), "TX:%lu", (unsigned long)d.txCount);
  oled.setCursor(62, 40); oled.print(buf);

  oled.setCursor(0, 52);
  oled.print((d.state.flags & FLAG_GAMEPAD_LIVE) ? "LIVE" : "NO PAD");
  oled.print("  ENC=MENU");
  oled.display();
}

void drawTrimPage(const DisplaySnapshot &d) {
  oled.clearDisplay();
  if (d.subState == SUB_BROWSE) {
    printCentered("TRIM", 0);
    printCentered(d.trimSel == 0 ? "> Left Stick"  : "  Left Stick",  18);
    printCentered(d.trimSel == 1 ? "> Right Stick" : "  Right Stick", 30);
    printCentered("L/R:page  Enc:select", 52);
  } else {
    printCentered(d.trimSel == 0 ? "LEFT STICK TRIM" : "RIGHT STICK TRIM", 0);
    int16_t tx = (d.trimSel == 0) ? d.cfg.trimLX : d.cfg.trimRX;
    int16_t ty = (d.trimSel == 0) ? d.cfg.trimLY : d.cfg.trimRY;
    char buf[16];
    snprintf(buf, sizeof(buf), "X: %d", tx);
    printCentered(buf, 14, 2);
    snprintf(buf, sizeof(buf), "Y: %d", ty);
    printCentered(buf, 32, 2);
    printCentered("U/D=Y L/R=X Enc=back", 54);
  }
  oled.display();
}

void drawInvertPage(const DisplaySnapshot &d) {
  oled.clearDisplay();
  printCentered("CHANNEL INVERT", 0);

  const char *names[4] = { "LX", "LY", "RX", "RY" };
  for (int8_t i = 0; i < 4; i++) {
    bool on = d.cfg.revMask & (1 << i);
    char buf[24];
    snprintf(buf, sizeof(buf), "%s%s %s", (i == d.invertSel) ? "> " : "  ",
              names[i], on ? "[ON]" : "[off]");
    printCentered(buf, 10 + i * 10);
  }
  printCentered("L/R:page Enc:toggle", 54);
  oled.display();
}

// THROTTLE CURVE page: plots the default (non-linear) throttle curve in
// a 50x50 box on the left - stick position on X, output on Y, both
// -1..+1 - with the dotted diagonal showing what a straight linear
// response would be. The set value is printed as a small number on the
// right.
void drawCurvePage(const DisplaySnapshot &d) {
  oled.clearDisplay();
  printCentered("THROTTLE CURVE", 0);

  const int16_t gx = 6, gy = 12, gw = 50, gh = 50;      // outer box
  const int16_t ix = gx + 1, iy = gy + 1;               // inner plot area
  const int16_t iw = gw - 2, ih = gh - 2;
  oled.drawRect(gx, gy, gw, gh, SSD1306_WHITE);

  // Dotted linear reference (bottom-left -> top-right).
  for (int16_t i = 0; i < iw; i += 3) {
    oled.drawPixel(ix + i, iy + (ih - 1) - i, SSD1306_WHITE);
  }

  // The actual curve, joined with short line segments so steep parts
  // stay continuous.
  const float e = d.cfg.throttleExpo / 100.0f;
  int16_t prevPy = 0;
  for (int16_t px = 0; px < iw; px++) {
    float x = -1.0f + 2.0f * px / (iw - 1);
    float y = throttleNormalCurve(x, e);
    int16_t py = (ih - 1) - (int16_t)lroundf((y + 1.0f) * 0.5f * (ih - 1));
    if (px == 0) oled.drawPixel(ix, iy + py, SSD1306_WHITE);
    else         oled.drawLine(ix + px - 1, iy + prevPy, ix + px, iy + py, SSD1306_WHITE);
    prevPy = py;
  }

  // Right-hand column: value + key hints (left-aligned, size 1).
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  char buf[16];
  oled.setCursor(66, 16);
  oled.print("Expo:");
  snprintf(buf, sizeof(buf), "%u%%", (unsigned)d.cfg.throttleExpo);
  oled.setCursor(66, 26);
  oled.print(buf);
  oled.setCursor(66, 40);
  oled.print("Knob: +/-5");
  oled.setCursor(66, 48);
  oled.print("U/D:  +/-1");
  oled.setCursor(66, 56);
  oled.print("L/R:  page");

  oled.display();
}

// ---------------------------------------------------------------------
// OLED task - runs on its own core (see the big comment above dispMux).
// All the slow I2C work happens here, completely off the radio/USB
// loop's critical path. vTaskDelay() (not delay()) so this task truly
// yields the core instead of busy-waiting.
// ---------------------------------------------------------------------
void oledTask(void *pvParameters) {
  const TickType_t period = pdMS_TO_TICKS(1000UL / OLED_REFRESH_HZ);
  for (;;) {
    DisplaySnapshot d;
    portENTER_CRITICAL(&dispMux);
    d = dispSnap;                 // one small struct copy - microseconds
    portEXIT_CRITICAL(&dispMux);

    if (d.screen == SCR_STATUS) {
      drawStatusScreen(d);
    } else {
      switch (d.currentPage) {
        case PAGE_TRIM:     drawTrimPage(d);     break;
        case PAGE_INVERT:   drawInvertPage(d);   break;
        case PAGE_CURVE:    drawCurvePage(d);    break;
        default: break;
      }
    }

    vTaskDelay(period);   // paces the task AND yields the core - no busy-wait
  }
}


static bool i2cDevicePresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static bool initOLED() {
  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(400000);

  uint8_t addr = 0;
  if (i2cDevicePresent(0x3C)) addr = 0x3C;
  else if (i2cDevicePresent(0x3D)) addr = 0x3D;
  else return false;

  // Adafruit_SSD1306 stores the address internally during begin().
  if (!oled.begin(SSD1306_SWITCHCAPVCC, addr)) return false;

  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print("OLED OK 0x");
  oled.println(addr, HEX);
  oled.display();
  delay(250);
  oled.clearDisplay();
  oled.display();
  return true;
}

void setup() {
#if DEBUG_PRINT
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println(F("=== nRF24 gamepad transmitter (send-on-change) ==="));
#endif

  memset(&state, 0, sizeof(state));
  state.lx = state.ly = state.rx = state.ry = 128;

  // ---- Layer-2 control panel: inputs + saved trims + OLED ----
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_LEFT, INPUT_PULLUP);
  pinMode(BTN_RIGHT, INPUT_PULLUP);
  pinMode(ENC_BTN, INPUT_PULLUP);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  lastEncState = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), encoderISR, CHANGE);

  loadConfig();

  // ---- OLED: detect both common SSD1306 I2C addresses (0x3C/0x3D) ----
  oledPresent = initOLED();
  if (oledPresent) {
    // Show WHY we just booted, on the OLED itself - no laptop needed.
    // Runs on every boot, including the one right after an unexpected
    // reset, so if the "menu resets to home screen" glitch is really a
    // brownout/panic/watchdog reboot, this is where you'll see it.
    esp_reset_reason_t rr = esp_reset_reason();
    const char *reason = "UNKNOWN";
    switch (rr) {
      case ESP_RST_POWERON:  reason = "POWER-ON";  break;
      case ESP_RST_BROWNOUT: reason = "BROWNOUT";  break;
      case ESP_RST_PANIC:    reason = "PANIC";     break;
      case ESP_RST_INT_WDT:  reason = "INT WDT";   break;
      case ESP_RST_TASK_WDT: reason = "TASK WDT";  break;
      case ESP_RST_SW:       reason = "SW RESET";  break;
      default: break;
    }
    oled.clearDisplay();
    printCentered("BOOT:", 20);
    printCentered(reason, 34, 2);
    oled.display();
    delay(1500);   // long enough to actually read it before the normal UI takes over

    oled.clearDisplay();
    oled.display();

    // Initialize the snapshot before the OLED task starts.
    memset(&dispSnap, 0, sizeof(dispSnap));
    dispSnap.screen = SCR_STATUS;
    dispSnap.currentPage = PAGE_TRIM;
    dispSnap.subState = SUB_BROWSE;
    dispSnap.cfg = cfg;
    dispSnap.state = state;
    dispSnap.txCount = txCount;

    // Arduino-ESP32's loopTask (this code) runs on core 1 by default, so
    // pin the OLED task to core 0 - guarantees it can never share a core
    // with, let alone stall, the radio/USB loop.
    xTaskCreatePinnedToCore(oledTask, "oledTask", 4096, NULL, 1, NULL, 0);
  }
#if DEBUG_PRINT
  else {
    Serial.println(F("OLED not found - check wiring/address (0x3C vs 0x3D)."));
  }
#endif

  usb.onHIDInput([](const EspUsbHostHIDInput &input) {
    size_t len = input.length;
    if (len == 0) return;
    if (len > sizeof(rawReport)) len = sizeof(rawReport);

    portENTER_CRITICAL(&reportMux);
    memcpy(rawReport, input.data, len);
    rawReportLen = (uint8_t)len;
    newReportFlag = true;
    portEXIT_CRITICAL(&reportMux);
  });

  if (!usb.begin()) {
#if DEBUG_PRINT
    Serial.printf("usb.begin() failed: %s\n", usb.lastErrorName());
    Serial.println(F("(Continuing - heartbeat will still transmit for RF testing.)"));
#endif
  }

  radioSpi.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);   // -1: RF24 owns CSN

  if (!radio.begin(&radioSpi)) {
#if DEBUG_PRINT
    Serial.println(F("nRF24 NOT RESPONDING - check wiring and power."));
#endif
    while (1) { delay(1000); }
  }

  radio.setChannel(NRF_CHANNEL);
  radio.setDataRate(NRF_DATARATE);
  radio.setPALevel(NRF_PA_LEVEL);
  radio.setAutoAck(false);
  radio.setRetries(0, 0);
  radio.setCRCLength(RF24_CRC_16);
  radio.setAddressWidth(5);
  radio.disableDynamicPayloads();
  radio.setPayloadSize(sizeof(GamepadPacket));
  radio.openWritingPipe(txAddress);
  radio.stopListening();

#if DEBUG_PRINT
  Serial.printf("Radio up: ch=%d  1Mbps  PA=LOW  payload=%u bytes\n",
                NRF_CHANNEL, (unsigned)sizeof(GamepadPacket));
  Serial.printf("Send-on-change: min gap=%dms  heartbeat=%dms\n",
                MIN_TX_INTERVAL_MS, HEARTBEAT_MS);
#endif
}

void loop() {
  // No usb.task() call - v2 pumps USB events in its own background task.

  // ---------------- 1. Absorb any new USB report ----------------
  if (newReportFlag) {
    uint8_t localReport[sizeof(rawReport)];
    uint8_t localLen;

    portENTER_CRITICAL(&reportMux);
    localLen = rawReportLen;
    memcpy(localReport, rawReport, localLen);
    newReportFlag = false;
    portEXIT_CRITICAL(&reportMux);

    if (localLen >= layout.rtByte + 1) {
      state.buttons = localReport[layout.btnLowByte] |
                      ((uint16_t)localReport[layout.btnHighByte] << 8);
      state.dpad = localReport[layout.dpadByte];
      rawLX = localReport[layout.lxByte];
      rawLY = localReport[layout.lyByte];
      rawRX = localReport[layout.rxByte];
      rawRY = localReport[layout.ryByte];
      state.lt   = localReport[layout.ltByte];
      state.rt   = localReport[layout.rtByte];
      state.flags |= FLAG_GAMEPAD_LIVE;
      lastReportMs = millis();
    }

    memcpy(dbgReport, localReport, localLen);
    dbgLen = localLen;
  }

  if (millis() - lastReportMs > 200) {
    state.flags &= ~FLAG_GAMEPAD_LIVE;
  }

  // ---------------- 1b. Control panel + trim/expo/reverse/arm ----------------
  // Polling buttons on every single loop spin is wasted work: between
  // radio sends this loop can run thousands of times/sec doing nothing
  // useful. 1kHz is far more than enough to catch a 25ms-debounced
  // button or an encoder detent, and it keeps GPIO reads off the
  // gamepad-read -> radio-send path the rest of the time.
  static uint32_t lastCtrlMs = 0;
  uint32_t ctrlNow = millis();
  if (ctrlNow - lastCtrlMs >= 1) {
    lastCtrlMs = ctrlNow;
    handleControls();
  }

  // ML held -> near-linear throttle curve. Released -> default expo curve.
  bool aggressiveHeld = (state.buttons & AGGRESSIVE_BTN_MASK) != 0;

#if THROTTLE_IS_LY
  state.lx = applyAxis(rawLX, cfg.trimLX, cfg.revMask & REV_LX_BIT, cfg.expo);
  state.ly = applyThrottle(rawLY, cfg.trimLY, cfg.revMask & REV_LY_BIT, aggressiveHeld);
  state.rx = applyAxis(rawRX, cfg.trimRX, cfg.revMask & REV_RX_BIT, cfg.expo);
  state.ry = applyAxis(rawRY, cfg.trimRY, cfg.revMask & REV_RY_BIT, cfg.expo);
#else
  state.lx = applyAxis(rawLX, cfg.trimLX, cfg.revMask & REV_LX_BIT, cfg.expo);
  state.ly = applyAxis(rawLY, cfg.trimLY, cfg.revMask & REV_LY_BIT, cfg.expo);
  state.rx = applyAxis(rawRX, cfg.trimRX, cfg.revMask & REV_RX_BIT, cfg.expo);
  state.ry = applyThrottle(rawRY, cfg.trimRY, cfg.revMask & REV_RY_BIT, aggressiveHeld);
#endif


  // ---------------- 2. Send-on-change, rate-capped, + heartbeat ----------------
  uint32_t now = millis();
  uint32_t sinceLastTx = now - lastTxMs;

  bool changed = !haveLastSent || payloadChanged(state, lastSent);
  bool shouldSendForChange    = changed && (sinceLastTx >= MIN_TX_INTERVAL_MS);
  bool shouldSendForHeartbeat = (sinceLastTx >= HEARTBEAT_MS);

  if (shouldSendForChange || shouldSendForHeartbeat) {
    lastTxMs = now;
    state.seq++;
    radio.write(&state, sizeof(state));
    lastSent = state;
    haveLastSent = true;
    txCount++;
  }

  // ---------------- 2b. Publish display snapshot (cheap - no I2C here) ----------------
  // loop() only ever does a tiny struct copy under a spinlock; oledTask
  // (on the other core) does the slow I2C part on its own schedule.
  if (oledPresent) {
    portENTER_CRITICAL(&dispMux);
    dispSnap.screen      = screen;
    dispSnap.currentPage = currentPage;
    dispSnap.subState    = subState;
    dispSnap.trimSel     = trimSel;
    dispSnap.invertSel   = invertSel;
    dispSnap.cfg         = cfg;
    dispSnap.txCount     = txCount;
    dispSnap.state       = state;
    portEXIT_CRITICAL(&dispMux);
  }

  // ---------------- 3. Debug output, rate limited (compiled out if DEBUG_PRINT=0) ----------------
#if DEBUG_PRINT
  if (now - lastPrintMs >= (1000 / DEBUG_HZ)) {
    lastPrintMs = now;

#if DEBUG_RAW_BYTES
    static uint8_t prev[sizeof(rawReport)];
    static uint8_t prevLen = 0;
    if (dbgLen != prevLen || memcmp(prev, dbgReport, dbgLen) != 0) {
      Serial.printf("RAW len=%2u  ", dbgLen);
      for (uint8_t i = 0; i < dbgLen; i++) Serial.printf("%02X ", dbgReport[i]);
      Serial.println();
      memcpy(prev, dbgReport, dbgLen);
      prevLen = dbgLen;
    }
#else
    Serial.printf("TX#%lu %s %s  btn=0x%04X dpad=%3u  LX=%3u LY=%3u  RX=%3u RY=%3u  LT=%3u RT=%3u\n",
                  (unsigned long)txCount,
                  (state.flags & FLAG_GAMEPAD_LIVE) ? "PAD" : "---",
                  (unsigned)state.buttons, state.dpad,
                  state.lx, state.ly, state.rx, state.ry,
                  state.lt, state.rt);
#endif
  }
#endif
}

/*
  OPTIONAL: v2's built-in gamepad parser, if you want to skip manual
  byte-offset mapping:
    usb.onGamepad([](const EspUsbHostGamepadEvent &event) { ... });
  See examples/HID/EspUsbHostGamepad in the library.
*/
