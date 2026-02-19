/* Edge Impulse Arduino examples
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// These sketches are tested with 2.0.4 ESP32 Arduino Core
// https://github.com/espressif/arduino-esp32/releases/tag/2.0.4

/* Includes ---------------------------------------------------------------- */
#define EI_CLASSIFIER_OBJECT_DETECTION_THRESHOLD 0.01f

#include <Wildsights_rhino_md_conf.5_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"

#include "esp_camera.h"

#include "esp_heap_caps.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sd_read_write.h"

#include "img_converters.h"



#define BOOT_BUTTON_PIN 0   // ESP32-S3 BOOT button is GPIO0 on most boards

SemaphoreHandle_t infer_sem = nullptr;

void IRAM_ATTR onBootButtonISR() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (infer_sem) {
    xSemaphoreGiveFromISR(infer_sem, &xHigherPriorityTaskWoken);
  }
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}


TaskHandle_t inferTaskHandle = nullptr;

static uint8_t *snapshot_buf = nullptr;
static size_t snapshot_buf_size = 0;

// Select camera model - find more camera models in camera_pins.h file here
// https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Camera/CameraWebServer/camera_pins.h

//#define CAMERA_MODEL_ESP_EYE // Has PSRAM
//#define CAMERA_MODEL_AI_THINKER // Has PSRAM

#define CAMERA_MODEL_ESP32S3_EYE //ESP32S3 Camera

#if defined(CAMERA_MODEL_ESP_EYE)
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    4
#define SIOD_GPIO_NUM    18
#define SIOC_GPIO_NUM    23

#define Y9_GPIO_NUM      36
#define Y8_GPIO_NUM      37
#define Y7_GPIO_NUM      38
#define Y6_GPIO_NUM      39
#define Y5_GPIO_NUM      35
#define Y4_GPIO_NUM      14
#define Y3_GPIO_NUM      13
#define Y2_GPIO_NUM      34
#define VSYNC_GPIO_NUM   5
#define HREF_GPIO_NUM    27
#define PCLK_GPIO_NUM    25

#elif defined(CAMERA_MODEL_ESP32S3_EYE)
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 15
#define SIOD_GPIO_NUM 4
#define SIOC_GPIO_NUM 5

#define Y2_GPIO_NUM 11
#define Y3_GPIO_NUM 9
#define Y4_GPIO_NUM 8
#define Y5_GPIO_NUM 10
#define Y6_GPIO_NUM 12
#define Y7_GPIO_NUM 18
#define Y8_GPIO_NUM 17
#define Y9_GPIO_NUM 16

#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM 7
#define PCLK_GPIO_NUM 13

#else
#error "Camera model not selected"
#endif

/* Constant defines -------------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define EI_CAMERA_FRAME_BYTE_SIZE                 3

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
static bool is_initialised = false;
//uint8_t *snapshot_buf; //points to the output of the capture

static camera_config_t camera_config = {
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

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_GRAYSCALE, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QVGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 12, //0-63 lower number means higher quality
    .fb_count = 1,       //if more than one, i2s runs in continuous mode. Use only with JPEG
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

/* Function definitions ------------------------------------------------------- */
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) ;


static void save_grayscale_as_jpeg_to_sd()
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ei_printf("Capture failed (null fb)\r\n");
        return;
    }

    // Expect grayscale framebuffer
    ei_printf("fb format=%d w=%d h=%d len=%u\r\n",
              (int)fb->format, fb->width, fb->height, (unsigned)fb->len);

    if (fb->format != PIXFORMAT_GRAYSCALE) {
        ei_printf("Expected GRAYSCALE, got format=%d (not saving)\r\n", (int)fb->format);
        esp_camera_fb_return(fb);
        return;
    }

    uint8_t *jpg_buf = nullptr;
    size_t jpg_len = 0;

    // Convert grayscale bytes -> JPEG
    // quality: 10(best) .. 63(worst) in some ESP codepaths, but fmt2jpg uses 1..100 style in many builds.
    // 80 is a good starting point.
    bool ok = fmt2jpg(
        fb->buf, fb->len,
        fb->width, fb->height,
        PIXFORMAT_GRAYSCALE,
        80,
        &jpg_buf, &jpg_len
    );

    esp_camera_fb_return(fb); // return fb ASAP once we’ve copied/converted

    if (!ok || !jpg_buf || jpg_len < 2) {
        ei_printf("fmt2jpg failed\r\n");
        if (jpg_buf) free(jpg_buf);
        return;
    }

    // Validate JPEG header (FF D8)
    if (!(jpg_buf[0] == 0xFF && jpg_buf[1] == 0xD8)) {
        ei_printf("JPEG header invalid: %02X %02X (not saving)\r\n", jpg_buf[0], jpg_buf[1]);
        free(jpg_buf);
        return;
    }

    int photo_index = readFileNum(SD_MMC, "/camera");
    if (photo_index == -1) {
        ei_printf("readFileNum failed\r\n");
        free(jpg_buf);
        return;
    }

    String path = "/camera/" + String(photo_index) + ".jpg";
    writejpg(SD_MMC, path.c_str(), jpg_buf, jpg_len);
    ei_printf("Saved %s (%u bytes)\r\n", path.c_str(), (unsigned)jpg_len);

    free(jpg_buf);
}


/* Creation of a task to run image inference ----------------------------------- */
void inferenceTask(void *param) {
    ei_printf("inferenceTask started on core %d\r\n", xPortGetCoreID());
    while (true){

        // Wait here until BOOT is pressed
        xSemaphoreTake(infer_sem, portMAX_DELAY);

        // Debounce + make sure it's really pressed
        vTaskDelay(pdMS_TO_TICKS(150));
        if (digitalRead(BOOT_BUTTON_PIN) != LOW) {
        continue;
        }

        // instead of wait_ms, we'll wait on the signal, this allows threads to cancel us...
        if (ei_sleep(5) != EI_IMPULSE_OK) {
            return;
        }

        ei::signal_t signal;
        signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
        signal.get_data = &ei_camera_get_data;

        ei_printf("capturing...\n");
        if (!ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf)) {
            ei_printf("Failed to capture image\r\n");
            return;
        }
        ei_printf("done capturing\n");

        Serial.printf("first 16 px: ");
        for (int i = 0; i < 16; i++) Serial.printf("%d ", snapshot_buf[i]);
        Serial.println();

        // Quick image sanity check
        uint8_t mn = 255, mx = 0;
        uint32_t sum = 0;
        for (size_t i = 0; i < snapshot_buf_size; i++) {
        uint8_t v = snapshot_buf[i];
        if (v < mn) mn = v;
        if (v > mx) mx = v;
        sum += v;
        }
        ei_printf("img stats: min=%u max=%u mean=%.1f\n", mn, mx, (float)sum / snapshot_buf_size);

        save_grayscale_as_jpeg_to_sd();

        // Run the classifier
        ei_impulse_result_t result = { 0 };

        if (!heap_caps_check_integrity_all(true)) {
            ei_printf("HEAP CORRUPTED after capture, before classifier!\n");
            while (1) vTaskDelay(pdMS_TO_TICKS(1000));
        }

        ei_printf("classifying...\n");
        ei_printf("TASK=%s watermark(words)=%u\r\n",
            pcTaskGetName(NULL),
            (unsigned)uxTaskGetStackHighWaterMark(NULL));
        ei_printf("snap ptr=%p size=%u first=%u\r\n",
            snapshot_buf, (unsigned)snapshot_buf_size,
            snapshot_buf ? snapshot_buf[0] : 0);

        ei_printf("freeHeap=%u freePsram=%u\n", ESP.getFreeHeap(), ESP.getFreePsram());

        ei_printf("signal.total_length=%u, expected=%u\r\n",
            (unsigned)signal.total_length,
            (unsigned)EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);


        //EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);

        uint32_t t0 = millis();
        EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
        uint32_t t1 = millis();

        ei_printf("run_classifier wall time: %lu ms\n", (unsigned long)(t1 - t0));

        if (err != EI_IMPULSE_OK) {
            ei_printf("ERR: Failed to run classifier (%d)\n", err);
            return;
        }
        ei_printf("finished classifying\n");

        // print the predictions
        ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
                    result.timing.dsp, result.timing.classification, result.timing.anomaly);

        #if EI_CLASSIFIER_OBJECT_DETECTION == 1

        ei_printf("Object detection bounding boxes:\r\n");

        size_t bb_count = 0;
        for (size_t i = 0; i < EI_CLASSIFIER_OBJECT_DETECTION_COUNT; i++) {
            auto bb = result.bounding_boxes[i];

            // Skip empty results
            if (bb.value <= 0.0f || bb.label == nullptr) continue;

            bb_count++;
            ei_printf("  #%u: %s (%.3f) x=%u y=%u w=%u h=%u\r\n",
                    (unsigned)i, bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
        }

        ei_printf("bb_count=%u (slots=%u)\r\n",
            (unsigned)bb_count, (unsigned)EI_CLASSIFIER_OBJECT_DETECTION_COUNT);

        #else
        ei_printf("Predictions:\r\n");
        for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
            ei_printf("  %s: ", ei_classifier_inferencing_categories[i]);
            ei_printf("%.5f\r\n", result.classification[i].value);
        }
        #endif

        // Print anomaly result (if it exists)
        #if EI_CLASSIFIER_HAS_ANOMALY
        ei_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
        #endif

        #if EI_CLASSIFIER_HAS_VISUAL_ANOMALY
        ei_printf("Visual anomalies:\r\n");
        for (uint32_t i = 0; i < result.visual_ad_count; i++) {
            ei_impulse_result_bounding_box_t bb = result.visual_ad_grid_cells[i];
            if (bb.value == 0) {
                continue;
            }
            ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                    bb.label,
                    bb.value,
                    bb.x,
                    bb.y,
                    bb.width,
                    bb.height);
        }
        #endif
    }
}

/**
* @brief      Arduino setup function
*/
void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    delay(2000);
    //comment out the below line to start inference immediately after upload

    ei_printf("Model input: %dx%d\n", EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT);
    ei_printf("RAW_SAMPLE_COUNT=%d\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT);
    ei_printf("OBJ_DET=%d\n", EI_CLASSIFIER_OBJECT_DETECTION);
    ei_printf("EI_CLASSIFIER_OBJECT_DETECTION_COUNT=%d\n", EI_CLASSIFIER_OBJECT_DETECTION_COUNT);
    ei_printf("EI_CLASSIFIER_LABEL_COUNT=%d\n", EI_CLASSIFIER_LABEL_COUNT);

    pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);          // BOOT is usually active-low
    infer_sem = xSemaphoreCreateBinary();

    // Trigger on press (falling edge)
    attachInterrupt(digitalPinToInterrupt(BOOT_BUTTON_PIN), onBootButtonISR, FALLING);

    sdmmcInit();
    createDir(SD_MMC, "/camera");
    listDir(SD_MMC, "/camera", 0);


    ei_printf("Edge Impulse Inferencing Demo");
    if (ei_camera_init() == false) {
        ei_printf("Failed to initialize Camera!\r\n");
        while (1) {ei_sleep(1000);}
    }
    else {
        ei_printf("Camera initialized\r\n");
    }
    
    ei_printf("INPUT_DATATYPE=%d\n", EI_CLASSIFIER_TFLITE_INPUT_DATATYPE);
    ei_printf("RAW_SAMPLE_COUNT=%d\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT);

    const uint32_t pixels = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    const uint32_t bpp = EI_CLASSIFIER_RAW_SAMPLE_COUNT / pixels;  // 1=gray, 3=RGB

    snapshot_buf_size = (size_t)EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT; // 9216 96x96
    snapshot_buf = (uint8_t*)heap_caps_malloc(snapshot_buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    ei_printf("pixels=%u RAW_SAMPLE_COUNT=%u bpp=%u snapshot_buf_size=%u\n",
          pixels, EI_CLASSIFIER_RAW_SAMPLE_COUNT, bpp, (unsigned)snapshot_buf_size);

    if (!snapshot_buf) {
    ei_printf("ERR: Failed to allocate snapshot buffer (%u bytes)\r\n", (unsigned)snapshot_buf_size);
    while (1) { ei_sleep(1000); }
    }

    ei_printf("RAW_SAMPLE_COUNT = %u snapshot_buf_size=%u\n", (unsigned)EI_CLASSIFIER_RAW_SAMPLE_COUNT, (unsigned)snapshot_buf_size);

    run_classifier_init();
    ei_printf("run_classifier_init() called");

    xTaskCreatePinnedToCore(inferenceTask, "inferenceTask", 32768, nullptr, 1, &inferTaskHandle, 1);

    ei_printf("\nStarting continious inference in 2 seconds...\n");
    ei_sleep(2000);
}

/**
* @brief      Get data and run inferencing
*
* @param[in]  debug  Get debug info if true
*/

/* Nothing runs in loop as it is all running in created task */
void loop()
{
    vTaskDelay(pdMS_TO_TICKS(1000));
}

/**
 * @brief   Setup image sensor & start streaming
 *
 * @retval  false if initialisation failed
 */
bool ei_camera_init(void) {

    if (is_initialised) return true;

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

//initialize the camera
esp_err_t err = esp_camera_init(&camera_config);
if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return false;
}

sensor_t * s = esp_camera_sensor_get();
// initial sensors are flipped vertically and colors are a bit saturated
if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, 0); // lower the saturation
}

#if defined(CAMERA_MODEL_M5STACK_WIDE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
#elif defined(CAMERA_MODEL_ESP_EYE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
    s->set_awb_gain(s, 1);
#endif

    is_initialised = true;
    return true;
}

/**
 * @brief      Stop streaming of sensor data
 */
void ei_camera_deinit(void) {

    //deinitialize the camera
    esp_err_t err = esp_camera_deinit();

    if (err != ESP_OK)
    {
        ei_printf("Camera deinit failed\n");
        return;
    }

    is_initialised = false;
    return;
}

/**
 * @brief      Capture, rescale and crop image
 *
 * @param[in]  img_width     width of output image
 * @param[in]  img_height    height of output image
 * @param[in]  out_buf       pointer to store output image, NULL may be used
 *                           if ei_camera_frame_buffer is to be used for capture and resize/cropping.
 *
 * @retval     false if not initialised, image captured, rescaled or cropped failed
 *
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    if (!is_initialised) {
        ei_printf("ERR: Camera is not initialized\r\n");
        return false;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ei_printf("Camera capture failed\r\n");
        return false;
    }

    const uint32_t pixels = img_width * img_height;
    const uint32_t bpp = EI_CLASSIFIER_RAW_SAMPLE_COUNT / pixels;   // 1 or 3

    // ---- GRAYSCALE framebuffer path ----
    if (fb->format == PIXFORMAT_GRAYSCALE) {
        uint8_t *p = fb->buf;

        for (uint32_t y = 0; y < img_height; y++) {
            uint32_t src_y = (uint32_t)((uint64_t)y * fb->height / img_height);
            for (uint32_t x = 0; x < img_width; x++) {
                uint32_t src_x = (uint32_t)((uint64_t)x * fb->width / img_width);
                uint8_t gray = p[src_y * fb->width + src_x];

                if (bpp == 1) {
                    out_buf[y * img_width + x] = gray;
                } else if (bpp == 3) {
                    size_t dst = (y * img_width + x) * 3;
                    out_buf[dst + 0] = gray;
                    out_buf[dst + 1] = gray;
                    out_buf[dst + 2] = gray;
                } else {
                    ei_printf("ERR: unsupported bpp=%u\r\n", (unsigned)bpp);
                    esp_camera_fb_return(fb);
                    return false;
                }
            }
        }

        esp_camera_fb_return(fb);
        return true;
    }

    // ---- RGB565 framebuffer path (your original logic) ----
    if (fb->format == PIXFORMAT_RGB565) {
        uint16_t *p = (uint16_t*)fb->buf;

        for (uint32_t y = 0; y < img_height; y++) {
            uint32_t src_y = (uint32_t)((uint64_t)y * fb->height / img_height);
            for (uint32_t x = 0; x < img_width; x++) {
                uint32_t src_x = (uint32_t)((uint64_t)x * fb->width / img_width);
                uint16_t px = p[src_y * fb->width + src_x];

                uint8_t r = ((px >> 11) & 0x1F) << 3;
                uint8_t g = ((px >> 5)  & 0x3F) << 2;
                uint8_t b = ( px        & 0x1F) << 3;

                if (bpp == 1) {
                    uint8_t luma = (uint8_t)((r * 30 + g * 59 + b * 11) / 100);
                    out_buf[y * img_width + x] = luma;
                } else if (bpp == 3) {
                    size_t dst = (y * img_width + x) * 3;
                    out_buf[dst + 0] = r;
                    out_buf[dst + 1] = g;
                    out_buf[dst + 2] = b;
                } else {
                    ei_printf("ERR: unsupported bpp=%u\r\n", (unsigned)bpp);
                    esp_camera_fb_return(fb);
                    return false;
                }
            }
        }

        esp_camera_fb_return(fb);
        return true;
    }

    // ---- Unsupported framebuffer format ----
    ei_printf("ERR: fb format %d not supported\r\n", (int)fb->format);
    esp_camera_fb_return(fb);
    return false;
}


static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    for (size_t i = 0; i < length; i++) {
        #if (EI_CLASSIFIER_TFLITE_INPUT_DATATYPE == EI_CLASSIFIER_DATATYPE_INT8)
            // int8 models typically expect -128..127
            out_ptr[i] = (float)((int)snapshot_buf[offset + i] - 128);

        #elif (EI_CLASSIFIER_TFLITE_INPUT_DATATYPE == EI_CLASSIFIER_DATATYPE_UINT8)
            // uint8 models expect 0..255
            out_ptr[i] = (float)snapshot_buf[offset + i];

        #else
            // float models typically expect 0..1
            out_ptr[i] = (float)snapshot_buf[offset + i] / 255.0f;
        #endif
    }
    return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif
