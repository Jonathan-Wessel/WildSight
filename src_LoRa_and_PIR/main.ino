#include "esp_camera.h"
#define CAMERA_MODEL_ESP32S3_EYE
#include "camera_pins.h"
#include "ws2812.h"
#include "sd_read_write.h"

#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <string.h>

#define BUTTON_PIN 0
#define GPIO_PIR 1
#define GPIO_IRLED 2
#define GPIO_LIGHTSENSOR 46

// LoRa pins
#define PIN_NSS  3   // CS/NSS
#define PIN_RST  14  // RESET
#define PIN_DIO0 20  // DIO0
#define PIN_DIO1 21  // DIO1

// Custom SPI pins
#define LORA_MISO 47
#define LORA_MOSI 19
#define LORA_SCK  45

// =================== LoRaWAN KEYS ===================
static const uint8_t JOIN_EUI_BE[8] = {
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};

static const uint8_t DEV_EUI_BE[8]  = {
  0x70,0xB3,0xD5,0x7E,0xD8,0x00,0x50,0x7F
};

static const uint8_t APP_KEY[16]    = {
  0x6E,0x2C,0xB9,0x5B,0x06,0x57,0x13,0x4B,
  0x9B,0xCC,0xEC,0x15,0xB6,0x5C,0x51,0x74
};

void os_getArtEui (u1_t* buf) {
  for (int i = 0; i < 8; i++) buf[i] = JOIN_EUI_BE[7 - i];
}

void os_getDevEui (u1_t* buf) {
  for (int i = 0; i < 8; i++) buf[i] = DEV_EUI_BE[7 - i];
}

void os_getDevKey (u1_t* buf) {
  memcpy(buf, APP_KEY, 16);
}

const lmic_pinmap lmic_pins = {
  .nss = PIN_NSS,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = PIN_RST,
  .dio = { PIN_DIO0, PIN_DIO1, LMIC_UNUSED_PIN }
};

static osjob_t sendjob;

// ---- Payload queue ----
static char pendingMsg[51];              // keep <= 51 for safety at lower DRs
static volatile bool havePendingMsg = false;

// Forward decl
void do_send(osjob_t*);

// Queue any string payload (ASCII)
bool sendString(const char* s) {
  if (s == nullptr) return false;

  size_t len = strnlen(s, sizeof(pendingMsg) - 1);
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

void LoRa_setup() {
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

void PIR_setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  Serial.println();

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(GPIO_PIR, INPUT);
  pinMode(GPIO_IRLED, OUTPUT);
  pinMode(GPIO_LIGHTSENSOR, INPUT);

  ws2812Init();
  sdmmcInit();

  // removeDir(SD_MMC, "/camera");
  createDir(SD_MMC, "/camera");
  listDir(SD_MMC, "/camera", 0);

  if (cameraSetup() == 1) {
    ws2812SetColor(2);
  } else {
    ws2812SetColor(1);
    return;
  }

  delay(10000);
}

int cameraSetup(void) {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG; // for streaming
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  // for larger pre-allocated frame buffer.
  if (psramFound()) {
    config.jpeg_quality = 10;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    // Limit the frame size when PSRAM is not available
    config.frame_size = FRAMESIZE_SVGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return 0;
  }

  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  s->set_vflip(s, 1);       // flip it back
  s->set_brightness(s, 1);  // up the brightness just a bit
  s->set_saturation(s, 0);  // lower the saturation

  Serial.println("Camera configuration complete!");
  return 1;
}

void PIR_Loop() {
  if (digitalRead(GPIO_PIR) == HIGH) {
    delay(20);

    if (digitalRead(GPIO_PIR) == HIGH) {
      ws2812SetColor(3);

      while (digitalRead(GPIO_PIR) == HIGH) {
        // wait for PIR to go low again
      }

      camera_fb_t * fb = nullptr;
      fb = esp_camera_fb_get();

      if (fb != nullptr) {
        int photo_index = readFileNum(SD_MMC, "/camera");
        if (photo_index != -1) {
          String path = "/camera/" + String(photo_index) + ".jpg";
          writejpg(SD_MMC, path.c_str(), fb->buf, fb->len);
        }
        esp_camera_fb_return(fb);
      } else {
        Serial.println("Camera capture failed.");
      }

      ws2812SetColor(2);
      queueSendRhinoDetected();
    }

    delay(5000);
  }
}

void setup() {
  PIR_setup();
  LoRa_setup();
}

void loop() {
  os_runloop_once();
  PIR_Loop();

  // Example hook point:
  // if (rhinoConfirmedThisFrame) queueSendRhinoDetected();
}
