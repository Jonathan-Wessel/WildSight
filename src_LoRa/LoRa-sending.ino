#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <string.h>

static const uint8_t JOIN_EUI_BE[8] = { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 };
static const uint8_t DEV_EUI_BE[8]  = { 0x70,0xB3,0xD5,0x7E,0xD8,0x00,0x50,0x7F };
static const uint8_t APP_KEY[16]    = {
  0x6E,0x2C,0xB9,0x5B,0x06,0x57,0x13,0x4B,
  0x9B,0xCC,0xEC,0x15,0xB6,0x5C,0x51,0x74
};

void os_getArtEui (u1_t* buf) { for (int i=0;i<8;i++) buf[i] = JOIN_EUI_BE[7-i]; }
void os_getDevEui (u1_t* buf) { for (int i=0;i<8;i++) buf[i] = DEV_EUI_BE[7-i]; }
void os_getDevKey (u1_t* buf) { memcpy(buf, APP_KEY, 16); }

// LoRa pins
static const uint8_t PIN_NSS  = 3;    // CS/NSS
static const uint8_t PIN_RST  = 14;   // RESET
static const uint8_t PIN_DIO0 = 20;   // DIO0
static const uint8_t PIN_DIO1 = 21;   // DIO1

// Custom SPI pins
static const uint8_t LORA_MISO = 47;
static const uint8_t LORA_MOSI = 48;
static const uint8_t LORA_SCK  = 45;

const lmic_pinmap lmic_pins = {
  .nss = PIN_NSS,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = PIN_RST,
  .dio = { PIN_DIO0, PIN_DIO1, LMIC_UNUSED_PIN }
};

static osjob_t sendjob;

// ---- Payload queue ----
static char pendingMsg[51];   // keep <= 51 for safety at lower DRs
static volatile bool havePendingMsg = false;

// Forward decl
void do_send(osjob_t*);

// Queue any string payload (ASCII)
bool sendString(const char* s) {
  if (s == nullptr) return false;

  size_t len = strnlen(s, sizeof(pendingMsg)-1);
  if (len == 0) return false;

  // Store it for sending
  memset(pendingMsg, 0, sizeof(pendingMsg));
  memcpy(pendingMsg, s, len);
  havePendingMsg = true;

  // Try immediate send (or it will retry from do_send)
  do_send(&sendjob);
  return true;
}

// Call this from your camera/model code when rhino is confirmed
void queueSendRhinoDetected() {
  sendString("Rhino detected");
}

void do_send(osjob_t*) {
  // Must be joined (LMIC.devaddr becomes non-zero after join)
  if (LMIC.devaddr == 0) {
    // not joined yet; keep message pending
    return;
  }

  // Radio busy?
  if (LMIC.opmode & OP_TXRXPEND) {
    // retry soon
    os_setTimedCallback(&sendjob, os_getTime() + ms2osticks(500), do_send);
    return;
  }

  if (!havePendingMsg) return;

  // Convert message to bytes (no null terminator)
  uint8_t buf[51];
  size_t len = strnlen(pendingMsg, sizeof(pendingMsg));
  memcpy(buf, pendingMsg, len);

  LMIC_setTxData2(/*port*/1, buf, (u1_t)len, /*confirmed*/0);

  Serial.print("Uplink queued: ");
  Serial.println(pendingMsg);

  havePendingMsg = false;
}

void onEvent(ev_t ev) {
  Serial.print("[LMIC] ");
  Serial.println((unsigned)ev);

  if (ev == EV_JOINING) Serial.println("Joining...");

  if (ev == EV_JOINED) {
    Serial.println("JOINED!");
    LMIC_setLinkCheckMode(0);

    // If something was queued before join completed, send it now
    do_send(&sendjob);
  }

  if (ev == EV_JOIN_FAILED || ev == EV_JOIN_TXCOMPLETE) {
    Serial.println("Join not accepted (keys/subband/pins/frequency-plan issue)");
  }

  if (ev == EV_TXCOMPLETE) {
    Serial.println("TX complete");

    // Optional: schedule periodic retry send if you want periodic telemetry.
    // For your case (event-based), you can remove this entirely.
    // os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(30), do_send);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, PIN_NSS);

  os_init();
  LMIC_reset();

  // US915 subband (adjust to match your gateway FSB)
  LMIC_selectSubBand(1);

  LMIC_setAdrMode(0);
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

  LMIC_startJoining();

  // --- TEST: comment this out once you hook camera trigger ---
  // queueSendRhinoDetected();
}

void loop() {
  os_runloop_once();

  // Example hook point:
  // if (rhinoConfirmedThisFrame) queueSendRhinoDetected();
}
