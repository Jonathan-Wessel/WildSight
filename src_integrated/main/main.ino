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
//#define EI_CLASSIFIER_OBJECT_DETECTION_THRESHOLD 0.5f
//#include <rhino_md_conf.5_second_class_inferencing.h>
#include <Wildsights_rhino_md_conf.5_inferencing.h>
//#include <Dog-Detector_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_heap_caps.h"

// =================== FreeRTOS ===================draw
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

//Camera Initialization Function
bool cameraInitGray() {
  if (cam_init_ok) return true;

  esp_err_t err = esp_camera_init(&cam_cfg);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  Serial.printf("Camera PID: 0x%02x\n", s->id.PID);

      if (s->id.PID == OV3660_PID) {
      s->set_vflip(s, 1); // flip it back
      s->set_brightness(s, 1); // up the brightness just a bit
      s->set_saturation(s, 0); // lower the saturation
    }

  cam_init_ok = true;
  return true;
}

// EI data feed
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
    size_t pixel_ix = offset;
    size_t pixels_left = length;

    while (pixels_left != 0) {
        out_ptr[0] = (float)snapshot_buf[pixel_ix];
        out_ptr++;
        pixel_ix++;
        pixels_left--;
    }

    return 0;
}

// Save latest GRAYSCALE frame as JPEG, optionally drawing a bbox first
bool saveGrayJpegWithBox(camera_fb_t *fb) {
  if (!fb) {
    Serial.println("saveGrayJpegWithBox: fb is null");
    return false;
  }

  if (fb->format != PIXFORMAT_GRAYSCALE) {
    Serial.printf("saveGrayJpegWithBox: wrong format %d\n", fb->format);
    return false;
  }

  uint8_t *jpg_buf = nullptr;
  size_t jpg_len = 0;

  Serial.printf("Encoding JPEG from grayscale: w=%u h=%u len=%u\n",
                fb->width, fb->height, fb->len);

  bool ok = fmt2jpg(
    fb->buf, fb->len,
    fb->width, fb->height,
    PIXFORMAT_GRAYSCALE,
    80,
    &jpg_buf, &jpg_len
  );

  Serial.printf("fmt2jpg ok=%d jpg_buf=%p jpg_len=%u freeHeap=%u\n",
                ok ? 1 : 0,
                jpg_buf,
                (unsigned)jpg_len,
                (unsigned)ESP.getFreeHeap());

  if (!ok || !jpg_buf || jpg_len < 2) {
    Serial.println("JPEG encode failed");
    if (jpg_buf) free(jpg_buf);
    return false;
  }

  int idx = readFileNum(SD_MMC, "/camera");
  if (idx < 0) {
    Serial.println("readFileNum failed");
    free(jpg_buf);
    return false;
  }

  String path = "/camera/" + String(idx) + ".jpg";

  bool write_ok = writejpg(SD_MMC, path.c_str(), jpg_buf, jpg_len);
  if (write_ok) {
    Serial.print("Saved: ");
    Serial.println(path);
  } else {
    Serial.print("Failed to save: ");
    Serial.println(path);
  }

  free(jpg_buf);
  return write_ok;
}

static bool getBestRhinoBox(const ei_impulse_result_t &result, ei_impulse_result_bounding_box_t &best) {
  best = {0};

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
  bool found = false;
  for (size_t i = 0; i < result.bounding_boxes_count; i++) {
    ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
    if (bb.label == nullptr) continue;

            ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                bb.label,
                bb.value,
                bb.x,
                bb.y,
                bb.width,
                bb.height);

    // Relaxed match: Check if "rhino" is anywhere in the label
    if (strcmp(bb.label, "rhino") == 0) {
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

static bool ei_resize_from_fb(camera_fb_t *fb, uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    if (!fb) {
        ei_printf("ERR: fb is null\n");
        return false;
    }

    if (fb->format != PIXFORMAT_GRAYSCALE) {
        ei_printf("ERR: expected GRAYSCALE fb, got %d\n", (int)fb->format);
        return false;
    }

    const uint32_t src_w = fb->width;
    const uint32_t src_h = fb->height;

    if (src_w == 0 || src_h == 0 || img_width == 0 || img_height == 0) {
        ei_printf("ERR: invalid dimensions\n");
        return false;
    }

    // FIT_SHORTEST:
    // Scale so the shortest source axis fills the destination,
    // then center-crop the overflow on the longer axis.
    const float scale_x = (float)img_width / (float)src_w;
    const float scale_y = (float)img_height / (float)src_h;
    const float scale = (scale_x > scale_y) ? scale_x : scale_y;

    const float scaled_w = src_w * scale;
    const float scaled_h = src_h * scale;

    const float crop_x_scaled = (scaled_w - img_width) * 0.5f;
    const float crop_y_scaled = (scaled_h - img_height) * 0.5f;

    for (uint32_t y = 0; y < img_height; y++) {
        for (uint32_t x = 0; x < img_width; x++) {
            // Destination pixel -> scaled-source space
            float sx_scaled = x + crop_x_scaled;
            float sy_scaled = y + crop_y_scaled;

            // Scaled-source space -> original source space
            float sx = sx_scaled / scale;
            float sy = sy_scaled / scale;

            // Nearest neighbor
            int src_x = (int)(sx + 0.5f);
            int src_y = (int)(sy + 0.5f);

            if (src_x < 0) src_x = 0;
            if (src_y < 0) src_y = 0;
            if (src_x >= (int)src_w) src_x = (int)src_w - 1;
            if (src_y >= (int)src_h) src_y = (int)src_h - 1;

            out_buf[y * img_width + x] = fb->buf[src_y * src_w + src_x];
        }
    }

    return true;
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

    for (int i = 0; i < 2; i++) {
      camera_fb_t *tmp = esp_camera_fb_get();
      if (tmp) esp_camera_fb_return(tmp);
      vTaskDelay(pdMS_TO_TICKS(10));
    }

    // instead of wait_ms, we'll wait on the signal, this allows threads to cancel us...
        if (ei_sleep(5) != EI_IMPULSE_OK) {
            return;
        }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
    signal.get_data = &ei_camera_get_data;

    //digitalWrite(GPIO_IRLED, HIGH);
    camera_fb_t *fb = esp_camera_fb_get();
      if (!fb) {
          Serial.printf("Camera capture failed\r\n");
          continue;
      }

      Serial.printf("fb format=%d w=%d h=%d len=%u\r\n",
              (int)fb->format, fb->width, fb->height, (unsigned)fb->len);

      Serial.printf("capturing...\n");
      if (!ei_resize_from_fb(fb, EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf)) {
          Serial.printf("Failed to resize image\r\n");
          esp_camera_fb_return(fb);
          continue;
      }
      Serial.printf("done capturing\n");

    Serial.printf("INFERENCE: running classifier\n");
    ei_impulse_result_t result = {0};
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);
    Serial.printf("INFERENCE: done classifier\n");
   
    if (err != EI_IMPULSE_OK) {
      Serial.printf("run_classifier failed (%d)\n", err);
      //ws2812SetColor(1);
      continue;
    }

    // ---- Decide rhino + bbox ----
    ei_impulse_result_bounding_box_t best;
    bool rhino = getBestRhinoBox(result, best);

    if (rhino && best.value >= 0.7) {
      Serial.printf("Rhino DETECTED ✅ label = %s : conf=%.3f box(x=%u y=%u w=%u h=%u)\n",
                    best.label, best.value, best.x, best.y, best.width, best.height);

      //ws2812SetColor(2);


      if (!saveGrayJpegWithBox(fb)) {
          Serial.println("Save failed");
      }

      // Send LoRa
      queueSendRhinoDetected();
    }
    else {
      Serial.println("No rhino.");
      //ws2812SetColor(1);
      //sendString("No Rhino");
      // Optional: save non-detections too
    }

    digitalWrite(GPIO_IRLED, LOW);

    // Cooldown
    vTaskDelay(pdMS_TO_TICKS(5000));

    esp_camera_fb_return(fb);
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
  digitalWrite(GPIO_IRLED, LOW);
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

  //PIR semaphore + ISR
  pir_sem = xSemaphoreCreateBinary();
  attachInterrupt(digitalPinToInterrupt(GPIO_PIR), onPirISR, RISING);

  // Inference task
  xTaskCreatePinnedToCore(inferenceTask, "inferenceTask", 32768, nullptr, 1, &inferTaskHandle, 1);

  Serial.println("Setup complete.");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(10000));
}
