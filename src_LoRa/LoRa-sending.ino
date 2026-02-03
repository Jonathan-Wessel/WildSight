#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <string.h>  

static const uint8_t JOIN_EUI_BE[8] = { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 };
static const uint8_t DEV_EUI_BE[8]  = { 0x70,0xB3,0xD5,0x7E,0xD8,0x00,0x50,0x7F }; // <--  70-B3-D5-7E-D8-00-50-7F
static const uint8_t APP_KEY[16]    = {
  0x6E,0x2C,0xB9,0x5B,0x06,0x57,0x13,0x4B,
  0x9B,0xCC,0xEC,0x15,0xB6,0x5C,0x51,0x74
}; // <-- 6E-2C-B9-5B-06-57-13-4B-9B-CC-EC-15-B6-5C-51-74

void os_getArtEui (u1_t* buf) { for (int i=0;i<8;i++) buf[i] = JOIN_EUI_BE[7-i]; }
void os_getDevEui (u1_t* buf) { for (int i=0;i<8;i++) buf[i] = DEV_EUI_BE[7-i]; }
void os_getDevKey (u1_t* buf) { memcpy(buf, APP_KEY, 16); }

static const uint8_t PIN_NSS  = 3;   // CS/NSS
static const uint8_t PIN_RST  = 5;   // RESET
static const uint8_t PIN_DIO0 = 4;   // DIO0 (G0 on Adafruit breakout)
static const uint8_t PIN_DIO1 = 6;   // DIO1 (G1)

const lmic_pinmap lmic_pins = {
  .nss = PIN_NSS,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = PIN_RST,
  .dio = { PIN_DIO0, PIN_DIO1, LMIC_UNUSED_PIN }
};

static osjob_t sendjob;

static const char MESSAGE[] = "Hello World";

void do_send(osjob_t*) {
  if (LMIC.opmode & OP_TXRXPEND) return;

  const size_t len = strlen(MESSAGE);        // bytes, no '\0'
  if (len == 0) {
    Serial.println("MESSAGE is empty, not sending");
    return;
  }

  // LMIC expects a non-const pointer type (xref2u1_t), so cast is needed.
  LMIC_setTxData2(/*port*/1, (uint8_t*)MESSAGE, (u1_t)len, /*confirmed*/0);
  Serial.println("Uplink queued");
}

void onEvent(ev_t ev) {
  Serial.print("[LMIC] ");
  Serial.println((unsigned)ev);

  if (ev == EV_JOINING) Serial.println("Joining...");
  if (ev == EV_JOINED) {
    Serial.println("JOINED!");
    LMIC_setLinkCheckMode(0);
    do_send(&sendjob);
  }

  if (ev == EV_JOIN_FAILED || ev == EV_JOIN_TXCOMPLETE) {
    Serial.println("Join not accepted (keys/subband/pins/frequency-plan issue)");
  }

  if (ev == EV_TXCOMPLETE) {
    Serial.println("TX complete");
    // schedule next uplink
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(30), do_send);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  // If your SPI pins are NOT the board defaults, you must do:
  // SPI.begin(SCK, MISO, MOSI, PIN_NSS);

  os_init();
  LMIC_reset();

  // ----- US915 FSB2 -----
  // Gateway is US915 FSB2 => LMIC subband typically 1 (0-based)
  LMIC_selectSubBand(1);

  LMIC_setAdrMode(0);           // keep simple at first
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

  LMIC_startJoining();
}

void loop() {
  os_runloop_once();  // DO NOT block with long delay()
}
