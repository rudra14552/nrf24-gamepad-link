/*
  RECEIVER (SERIAL DEBUG ONLY) - decodes and prints every field of the
  11-byte GamepadPacket sent by nrf24-gamepad-link.ino.

  No PWM, no digital outputs, no failsafe - this sketch's only job is
  to show you exactly what's arriving over the air: every button,
  the dpad direction, all 4 stick axes, both triggers, the live/armed
  flags, seq number, and basic link health (drop count / link lost).

  PACKET FORMAT (must match the transmitter byte-for-byte, 11 bytes):
    seq      (1 byte)  - increments per packet actually sent
    flags    (1 byte)  - bit0 = GAMEPAD_LIVE, bit1 = ARMED
    buttons  (2 bytes) - bit0..12 = X,A,B,Y,LB,RB,LTclick,RTclick,
                          SELECT,START,ML,MR,HOME
    dpad     (1 byte)  - 8=neutral, 0=up, 2=right, 4=down, 6=left
    lx,ly    (1 byte each) - left stick,  0-255, 128=center
    rx,ry    (1 byte each) - right stick, 0-255, 128=center
    lt,rt    (1 byte each) - analog trigger pressure, 0-255

  Wiring: same nRF24 module/pins as the original receiver.ino
  (CE->9, CSN->10 on an Arduino Nano/Uno). Change PIN_CE/PIN_CSN below
  if you're wiring this onto different hardware.

  Radio settings (channel/datarate/address) MUST match the
  transmitter exactly, or you'll get nothing/garbage.
*/

#include <SPI.h>
#include <RF24.h>

// === nRF24 setup (must match transmitter's channel/datarate/PA/address) ===
#define PIN_CE   9
#define PIN_CSN  10
RF24 radio(PIN_CE, PIN_CSN);

#define NRF_CHANNEL   76
#define NRF_DATARATE  RF24_1MBPS      // MUST match transmitter
const uint8_t rxAddress[6] = "CTRL1"; // MUST match transmitter's txAddress

// === Packet (must match transmitter exactly, byte for byte) ===
#pragma pack(push, 1)
struct GamepadPacket {
  uint8_t  seq;
  uint8_t  flags;     // bit0 = GAMEPAD_LIVE, bit1 = ARMED
  uint16_t buttons;
  uint8_t  dpad;
  uint8_t  lx, ly;
  uint8_t  rx, ry;
  uint8_t  lt, rt;
};
#pragma pack(pop)
static_assert(sizeof(GamepadPacket) == 11, "packet must stay 11 bytes");

#define FLAG_GAMEPAD_LIVE  0x01
#define FLAG_ARMED         0x02

// Button bit positions - keep in sync with transmitter
#define BTN_X      0x0001
#define BTN_A      0x0002
#define BTN_B      0x0004
#define BTN_Y      0x0008
#define BTN_LB     0x0010
#define BTN_RB     0x0020
#define BTN_LTCLK  0x0040
#define BTN_RTCLK  0x0080
#define BTN_SELECT 0x0100
#define BTN_START  0x0200
#define BTN_ML     0x0400
#define BTN_MR     0x0800
#define BTN_HOME   0x1000

GamepadPacket pkt;

// === Link tracking (debug only, no failsafe outputs) ===
uint32_t lastReceiveMs = 0;
const uint16_t LINK_LOST_MS = 500;
uint8_t  lastSeq = 0;
bool     haveLastSeq = false;
uint32_t droppedCount = 0;
uint32_t packetCount = 0;

const char* dpadName(uint8_t d) {
  switch (d) {
    case 8: return "neutral";
    case 0: return "up";
    case 2: return "right";
    case 4: return "down";
    case 6: return "left";
    default: return "diag/?";
  }
}

void printButton(uint16_t buttons, uint16_t mask, const char* name) {
  if (buttons & mask) {
    Serial.print(name);
    Serial.print(' ');
  }
}

void setup() {
  Serial.begin(57600);

  if (!radio.begin()) {
    Serial.println(F("nRF24 not responding - check wiring!"));
    while (1) { delay(1000); }
  }
  radio.setChannel(NRF_CHANNEL);
  radio.setDataRate(NRF_DATARATE);
  radio.setPALevel(RF24_PA_LOW);       // plain receiver module, not the PA/LNA E01
  radio.setAutoAck(false);             // MUST match transmitter
  radio.setRetries(0, 0);
  radio.setCRCLength(RF24_CRC_16);
  radio.setAddressWidth(5);
  radio.disableDynamicPayloads();
  radio.setPayloadSize(sizeof(GamepadPacket));
  radio.openReadingPipe(0, rxAddress);
  radio.startListening();

  Serial.println(F("RX ready - serial debug only, all fields decoded"));
  lastReceiveMs = millis();
}

void loop() {
  uint32_t now = millis();

  if (radio.available()) {
    radio.read(&pkt, sizeof(pkt));
    lastReceiveMs = now;
    packetCount++;

    if (haveLastSeq) {
      uint8_t expected = lastSeq + 1;
      if (pkt.seq != expected) droppedCount++;
    }
    lastSeq = pkt.seq;
    haveLastSeq = true;

    bool live  = (pkt.flags & FLAG_GAMEPAD_LIVE) != 0;
    bool armed = (pkt.flags & FLAG_ARMED) != 0;

    Serial.print(F("#"));
    Serial.print(packetCount);
    Serial.print(F("  seq="));
    Serial.print(pkt.seq);
    Serial.print(F("  "));
    Serial.print(live ? F("LIVE") : F("heartbeat"));
    Serial.print(F("  "));
    Serial.print(armed ? F("ARMED") : F("safe"));
    Serial.print(F("  drop="));
    Serial.print(droppedCount);

    Serial.print(F("  | LX="));
    Serial.print(pkt.lx);
    Serial.print(F(" LY="));
    Serial.print(pkt.ly);
    Serial.print(F(" RX="));
    Serial.print(pkt.rx);
    Serial.print(F(" RY="));
    Serial.print(pkt.ry);
    Serial.print(F(" LT="));
    Serial.print(pkt.lt);
    Serial.print(F(" RT="));
    Serial.print(pkt.rt);

    Serial.print(F("  | dpad="));
    Serial.print(dpadName(pkt.dpad));

    Serial.print(F("  | buttons[0x"));
    if (pkt.buttons < 0x1000) Serial.print('0');
    if (pkt.buttons < 0x0100) Serial.print('0');
    if (pkt.buttons < 0x0010) Serial.print('0');
    Serial.print(pkt.buttons, HEX);
    Serial.print(F("]: "));
    printButton(pkt.buttons, BTN_X,      "X");
    printButton(pkt.buttons, BTN_A,      "A");
    printButton(pkt.buttons, BTN_B,      "B");
    printButton(pkt.buttons, BTN_Y,      "Y");
    printButton(pkt.buttons, BTN_LB,     "LB");
    printButton(pkt.buttons, BTN_RB,     "RB");
    printButton(pkt.buttons, BTN_LTCLK,  "LTclick");
    printButton(pkt.buttons, BTN_RTCLK,  "RTclick");
    printButton(pkt.buttons, BTN_SELECT, "SELECT");
    printButton(pkt.buttons, BTN_START,  "START");
    printButton(pkt.buttons, BTN_ML,     "ML");
    printButton(pkt.buttons, BTN_MR,     "MR");
    printButton(pkt.buttons, BTN_HOME,   "HOME");
    if (pkt.buttons == 0) Serial.print(F("(none)"));

    Serial.println();
  }

  if (haveLastSeq && (now - lastReceiveMs > LINK_LOST_MS)) {
    static uint32_t lastLostPrintMs = 0;
    if (now - lastLostPrintMs > 1000) {   // don't spam - once a second
      lastLostPrintMs = now;
      Serial.println(F("--- LINK LOST (no packet in >500ms) ---"));
    }
  }
}
