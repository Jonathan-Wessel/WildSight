// =================== Camera / SD / LEDs ===================
#include "esp_camera.h"
#define CAMERA_MODEL_ESP32S3_EYE
#include "camera_pins.h"
//#include "ws2812.h"
#include "sd_read_write.h"
#include "img_converters.h"   // fmt2jpg()

// =================== Arduino / LMIC / SPI ===================
#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <string.h>

// =================== Edge Impulse ===================
#define EI_CLASSIFIER_OBJECT_DETECTION_THRESHOLD 0.01f
#include <Wildsights_rhino_md_conf.5_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_heap_caps.h"

// =================== FreeRTOS ===================
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// =================== GPIO ===================
#define GPIO_PIR         1
#define GPIO_IRLED       2
#define GPIO_LIGHTSENSOR 46   // don’t share with NSS etc.

// LoRa pins
#define PIN_NSS   3
#define PIN_RST   14
#define PIN_DIO0  20
#define PIN_DIO1  21

// Custom SPI pins for LoRa radio
#define LORA_MISO  47
#define LORA_MOSI  19
#define LORA_SCK   45

// =================== LoRaWAN KEYS ===================
static const uint8_t JOIN_EUI_BE[8] = { 0,0,0,0,0,0,0,0 };
static const uint8_t DEV_EUI_BE[8]  = { 0x70,0xB3,0xD5,0x7E,0xD8,0x00,0x50,0x7F };
static const uint8_t APP_KEY[16]    = {
  0x6E,0x2C,0xB9,0x5B,0x06,0x57,0x13,0x4B,
  0x9B,0xCC,0xEC,0x15,0xB6,0x5C,0x51,0x74
};

void os_getArtEui (u1_t* buf) { for (int i=0;i<8;i++) buf[i] = JOIN_EUI_BE[7-i]; }
void os_getDevEui (u1_t* buf) { for (int i=0;i<8;i++) buf[i] = DEV_EUI_BE[7-i]; }
void os_getDevKey (u1_t* buf) { memcpy(buf, APP_KEY, 16); }

// LMIC pin mapping
const lmic_pinmap lmic_pins = {
  .nss = PIN_NSS,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = PIN_RST,
  .dio = { PIN_DIO0, PIN_DIO1, LMIC_UNUSED_PIN }
};

static osjob_t sendjob;

// =================== LoRa payload queue ===================
static char pendingMsg[51];
static volatile bool havePendingMsg = false;

void do_send(osjob_t*);

bool sendString(const char* s) {
  if (!s) return false;
  size_t len = strnlen(s, sizeof(pendingMsg)-1);
  if (len == 0) return false;

  memset(pendingMsg, 0, sizeof(pendingMsg));
  memcpy(pendingMsg, s, len);
  havePendingMsg = true;

  do_send(&sendjob);
  return true;
}

void queueSendRhinoDetected() {
  sendString("Rhino detected");
}

void do_send(osjob_t*) {
  if (LMIC.devaddr == 0) return;          // not joined yet
  if (LMIC.opmode & OP_TXRXPEND) {        // radio busy
    os_setTimedCallback(&sendjob, os_getTime() + ms2osticks(300), do_send);
    return;
  }
  if (!havePendingMsg) return;

  uint8_t buf[51];
  size_t len = strnlen(pendingMsg, sizeof(pendingMsg));
  memcpy(buf, pendingMsg, len);

  LMIC_setTxData2(1, buf, (u1_t)len, 0);
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
    do_send(&sendjob);
  }
  if (ev == EV_JOIN_FAILED) {
    Serial.println("Join failed (keys/subband/pins/frequency-plan issue)");
  }
  if (ev == EV_TXCOMPLETE) {
    Serial.println("TX complete");
  }
}

// =================== Bounding box draw (GRAYSCALE) ===================
static inline void putPixelGray(uint8_t* img, int w, int h, int x, int y, uint8_t v) {
  if (x < 0 || y < 0 || x >= w || y >= h) return;
  img[y * w + x] = v;
}

static void drawRectGray(uint8_t* img, int w, int h, int x, int y, int rw, int rh, uint8_t v) {
  if (rw <= 0 || rh <= 0) return;

  if (x < 0) { rw += x; x = 0; }
  if (y < 0) { rh += y; y = 0; }
  if (x + rw > w) rw = w - x;
  if (y + rh > h) rh = h - y;
  if (rw <= 0 || rh <= 0) return;

  for (int i = x; i < x + rw; i++) {
    putPixelGray(img, w, h, i, y, v);
    putPixelGray(img, w, h, i, y + rh - 1, v);
  }
  for (int j = y; j < y + rh; j++) {
    putPixelGray(img, w, h, x, j, v);
    putPixelGray(img, w, h, x + rw - 1, j, v);
  }
}

// =================== Camera config (GRAYSCALE QVGA) ===================
static bool cam_init_ok = false;

static camera_config_t cam_cfg = {
  .pin_pwdn = PWDN_GPIO_NUM,
  .pin_reset = RESET_GPIO_NUM,
  .pin_xclk = XCLK_GPIO_NUM,
  .pin_sscb_sda = SIOD_GPIO_NUM,
  .pin_sscb_scl = SIOC_GPIO_NUM,

  .pin_d7 = Y9_GPIO_NUM,
  .pin_d6 = Y8_GPIO_NUM,
  .pin_d5 = Y7_GPIO_NUM,
  .pin_d4 = Y6_GPIO_NUM,
  .pin_d3 = Y5_GPIO_NUM,
  .pin_d2 = Y4_GPIO_NUM,
  .pin_d1 = Y3_GPIO_NUM,
  .pin_d0 = Y2_GPIO_NUM,
  .pin_vsync = VSYNC_GPIO_NUM,
  .pin_href = HREF_GPIO_NUM,
  .pin_pclk = PCLK_GPIO_NUM,

  .xclk_freq_hz = 20000000,
  .ledc_timer = LEDC_TIMER_0,
  .ledc_channel = LEDC_CHANNEL_0,

  .pixel_format = PIXFORMAT_GRAYSCALE,
  .frame_size   = FRAMESIZE_QVGA,        // 320x240
  .jpeg_quality = 12,
  .fb_count     = 1,
  .fb_location  = CAMERA_FB_IN_PSRAM,
  .grab_mode    = CAMERA_GRAB_WHEN_EMPTY,
};

static uint8_t *snapshot_buf = nullptr;
static size_t snapshot_buf_size = 0;

bool cameraInitGray() {
  if (cam_init_ok) return true;

  esp_err_t err = esp_camera_init(&cam_cfg);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, 0);
  }

  cam_init_ok = true;
  return true;
}

// EI data feed
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
  if (!snapshot_buf) return -1;
  if (offset + length > snapshot_buf_size) {
    Serial.printf("EI get_data OOB: off=%u len=%u size=%u\n",
                  (unsigned)offset, (unsigned)length, (unsigned)snapshot_buf_size);
    return -1;
  }

  for (size_t i = 0; i < length; i++) {
#if (EI_CLASSIFIER_TFLITE_INPUT_DATATYPE == EI_CLASSIFIER_DATATYPE_INT8)
    out_ptr[i] = (float)((int)snapshot_buf[offset + i] - 128);
#elif (EI_CLASSIFIER_TFLITE_INPUT_DATATYPE == EI_CLASSIFIER_DATATYPE_UINT8)
    out_ptr[i] = (float)snapshot_buf[offset + i];
#else
    out_ptr[i] = (float)snapshot_buf[offset + i] / 255.0f;
#endif
  }
  return 0;
}


bool captureForEI(uint32_t out_w, uint32_t out_h, uint8_t *out_buf) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return false;
  }

  if (fb->format != PIXFORMAT_GRAYSCALE) {
    Serial.printf("Unexpected fb format=%d\n", (int)fb->format);
    esp_camera_fb_return(fb);
    return false;
  }

  // Calculate center crop dimensions to preserve the AI model's aspect ratio
  float target_ratio = (float)out_w / (float)out_h;
  float src_ratio = (float)fb->width / (float)fb->height;

  int crop_w = fb->width;
  int crop_h = fb->height;
  int offset_x = 0;
  int offset_y = 0;

  if (src_ratio > target_ratio) {
    // Source is wider than needed (e.g., 320x240 source, 96x96 target)
    // We crop the left and right sides.
    crop_w = (int)(fb->height * target_ratio);
    offset_x = (fb->width - crop_w) / 2;
  } else if (src_ratio < target_ratio) {
    // Source is taller than needed. We crop the top and bottom.
    crop_h = (int)(fb->width / target_ratio);
    offset_y = (fb->height - crop_h) / 2;
  }

  // Downsample ONLY the cropped region into the Edge Impulse buffer
  for (uint32_t y = 0; y < out_h; y++) {
    uint32_t src_y = offset_y + (y * crop_h) / out_h;
    for (uint32_t x = 0; x < out_w; x++) {
      uint32_t src_x = offset_x + (x * crop_w) / out_w;
      
      // Grab the grayscale pixel and place it in the EI buffer
      out_buf[y * out_w + x] = fb->buf[src_y * fb->width + src_x];
    }
  }

  esp_camera_fb_return(fb);
  return true;
}

// Save latest GRAYSCALE frame as JPEG, optionally drawing a bbox first
bool saveGrayJpegWithBox(int x, int y, int w, int h, bool drawBox) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return false;

  if (fb->format != PIXFORMAT_GRAYSCALE) {
    esp_camera_fb_return(fb);
    return false;
  }

  if (drawBox) {
    drawRectGray(fb->buf, fb->width, fb->height, x, y, w, h, 255);
  }

  uint8_t *jpg_buf = nullptr;
  size_t jpg_len = 0;

  bool ok = fmt2jpg(
    fb->buf, fb->len,
    fb->width, fb->height,
    PIXFORMAT_GRAYSCALE,
    80,
    &jpg_buf, &jpg_len
  );

  esp_camera_fb_return(fb);

  if (!ok || !jpg_buf || jpg_len < 2) {
    if (jpg_buf) free(jpg_buf);
    return false;
  }

  int idx = readFileNum(SD_MMC, "/camera");
  if (idx < 0) {
    free(jpg_buf);
    return false;
  }

  String path = "/camera/" + String(idx) + ".jpg";
  writejpg(SD_MMC, path.c_str(), jpg_buf, jpg_len);
  Serial.print("Saved: ");
  Serial.println(path);

  free(jpg_buf);
  return true;
}

static bool getBestRhinoBox(const ei_impulse_result_t &result, ei_impulse_result_bounding_box_t &best) {
  best = {0};

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
  bool found = false;
  for (size_t i = 0; i < EI_CLASSIFIER_OBJECT_DETECTION_COUNT; i++) {
    auto bb = result.bounding_boxes[i];
    if (!bb.label) continue;
    
    // Ignore anything below your threshold (currently 0.01f)
    if (bb.value < EI_CLASSIFIER_OBJECT_DETECTION_THRESHOLD) continue;

    // Relaxed match: Check if "rhino" or "Rhino" is anywhere in the label
    if (strstr(bb.label, "rhino") != nullptr || strstr(bb.label, "Rhino") != nullptr) {
      if (!found || bb.value > best.value) {
        best = bb;
        found = true;
      }
    }
  }
  return found;
#else
  (void)result;
  return false;
#endif
}

// =================== PIR semaphore + task ===================
SemaphoreHandle_t pir_sem = nullptr;
TaskHandle_t inferTaskHandle = nullptr;

void IRAM_ATTR onPirISR() {
  BaseType_t hp = pdFALSE;
  if (pir_sem) xSemaphoreGiveFromISR(pir_sem, &hp);
  if (hp) portYIELD_FROM_ISR();
}

void inferenceTask(void *param) {
  (void)param;
  Serial.printf("inferenceTask running on core %d\n", xPortGetCoreID());

  while (true) {
    xSemaphoreTake(pir_sem, portMAX_DELAY);

    // Debounce
    vTaskDelay(pdMS_TO_TICKS(50));
    if (digitalRead(GPIO_PIR) != HIGH) continue;

    //ws2812SetColor(3);

    // ---- EI capture + classify ----
    Serial.println("INFERENCE: starting capture");
    if (!captureForEI(EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf)) {
      Serial.println("INFERENCE: capture failed");
      //ws2812SetColor(1);
      continue;
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
    signal.get_data = &ei_camera_get_data;

    Serial.println("INFERENCE: running classifier");
    ei_impulse_result_t result = {0};
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);
    Serial.println("INFERENCE: done classifier");
   
    if (err != EI_IMPULSE_OK) {
      Serial.printf("run_classifier failed (%d)\n", err);
      //ws2812SetColor(1);
      continue;
    }

    // ---- Decide rhino + bbox ----
    ei_impulse_result_bounding_box_t best;
    bool rhino = getBestRhinoBox(result, best);

    if (rhino) {
      Serial.printf("RHINO DETECTED ✅ conf=%.3f box(x=%u y=%u w=%u h=%u)\n",
                    best.value, best.x, best.y, best.width, best.height);
      //ws2812SetColor(2);

      // Scale bbox from model input -> 320x240 framebuffer
      /*
      const int FB_W = 320;
      const int FB_H = 240;
      const int IN_W = EI_CLASSIFIER_INPUT_WIDTH;
      const int IN_H = EI_CLASSIFIER_INPUT_HEIGHT;

      int sx = (best.x * FB_W) / IN_W;
      int sy = (best.y * FB_H) / IN_H;
      int sw = (best.width  * FB_W) / IN_W;
      int sh = (best.height * FB_H) / IN_H;
      */
      // Scale bbox from model input -> cropped region -> 320x240 framebuffer
      const int FB_W = 320;
      const int FB_H = 240;
      const int IN_W = EI_CLASSIFIER_INPUT_WIDTH;
      const int IN_H = EI_CLASSIFIER_INPUT_HEIGHT;

      // 1. Re-calculate the crop dimensions used during capture
      float target_ratio = (float)IN_W / (float)IN_H;
      float src_ratio = (float)FB_W / (float)FB_H;

      int crop_w = FB_W;
      int crop_h = FB_H;
      int offset_x = 0;
      int offset_y = 0;

      if (src_ratio > target_ratio) {
        crop_w = (int)(FB_H * target_ratio);
        offset_x = (FB_W - crop_w) / 2;
      } else if (src_ratio < target_ratio) {
        crop_h = (int)(FB_W / target_ratio);
        offset_y = (FB_H - crop_h) / 2;
      }

      // 2. Map the AI's bounding box back onto the uncropped 320x240 photo
      int sx = offset_x + (best.x * crop_w) / IN_W;
      int sy = offset_y + (best.y * crop_h) / IN_H;
      int sw = (best.width * crop_w) / IN_W;
      int sh = (best.height * crop_h) / IN_H;
      // Save JPEG with bbox overlay
      if (!saveGrayJpegWithBox(sx, sy, sw, sh, true)) {
        Serial.println("Save failed");
      }

      // Send LoRa
      queueSendRhinoDetected();
    }
    else {
      Serial.println("No rhino.");
      //ws2812SetColor(1);
      sendString("No Rhino");
      // Optional: save non-detections too
      saveGrayJpegWithBox(0,0,0,0,false);
    }

    // Cooldown
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

// =================== Setup / Loop ===================
void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.printf("PSRAM found: %s\n", psramFound() ? "YES" : "NO");
  Serial.printf("Free heap: %u\n", ESP.getFreeHeap());
  Serial.printf("Free PSRAM: %u\n", ESP.getFreePsram());

  pinMode(GPIO_PIR, INPUT);
  pinMode(GPIO_IRLED, OUTPUT);
  pinMode(GPIO_LIGHTSENSOR, INPUT);

  //ws2812Init();

  // SD
  sdmmcInit();
  createDir(SD_MMC, "/camera");
  listDir(SD_MMC, "/camera", 0);

  // Camera
  if (!cameraInitGray()) {
    Serial.println("Camera init failed");
    while (1) delay(1000);
  }

  // EI buffer: for grayscale models RAW_SAMPLE_COUNT is usually W*H
  snapshot_buf_size = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
  snapshot_buf = (uint8_t*)heap_caps_malloc(snapshot_buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!snapshot_buf) {
    Serial.println("Failed to allocate snapshot buffer");
    while (1) delay(1000);
  }
  run_classifier_init();

  // LoRa
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, PIN_NSS);
  os_init();
  LMIC_reset();
  LMIC_selectSubBand(1);
  LMIC_setAdrMode(0);
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
  LMIC_startJoining();

  // PIR semaphore + ISR
  pir_sem = xSemaphoreCreateBinary();
  attachInterrupt(digitalPinToInterrupt(GPIO_PIR), onPirISR, RISING);

  // Inference task
  xTaskCreatePinnedToCore(inferenceTask, "inferenceTask", 32768, nullptr, 1, &inferTaskHandle, 1);

  Serial.println("Setup complete.");
}

void loop() {
  os_runloop_once();
}
