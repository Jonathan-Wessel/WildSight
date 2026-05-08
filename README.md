# WildSights Rhino Detection Camera Node

This project is an ESP32-S3 wildlife monitoring camera node that detects rhinos using an Edge Impulse object detection model. The device is triggered by a PIR motion sensor, captures a grayscale image, runs inference on-device, saves the captured image to an SD card, and sends a LoRaWAN uplink message with the detection result.

The project is currently configured for the `ESP32S3-EYE` camera model.

---

## Features

- PIR motion-triggered image capture
- ESP32-S3 camera support
- Edge Impulse object detection
- Rhino detection using bounding box results
- Grayscale QVGA camera capture
- SD card image saving as JPEG
- LoRaWAN uplinks using LMIC
- IR LED control for low-light image capture
- FreeRTOS task for image capture and inference

---

## Important Board Manager Version

Use the following ESP32 board package version:

```text
ESP32 by Espressif Systems version 2.0.17
```

In the Arduino IDE:

1. Open **Tools > Board > Boards Manager**
2. Search for **esp32**
3. Install **esp32 by Espressif Systems**
4. Select version **2.0.17**

Using a newer or different ESP32 board package version may cause compatibility issues with the camera, PSRAM, SD card, or libraries.

---

## Hardware Requirements

- ESP32-S3 camera board
- ESP32S3-EYE compatible camera pin configuration
- PIR motion sensor
- IR LED or IR illuminator
- Light sensor
- SD card
- LoRa radio module
- LoRaWAN gateway
- LoRaWAN network server, such as The Things Network

---

## Software Requirements

- Arduino IDE
- ESP32 Arduino board package version `2.0.17`
- MCCI LoRaWAN LMIC library
- Edge Impulse Arduino library exported from the Edge Impulse project
- ESP32 camera support
- SD_MMC support

The sketch also requires these project files:

```cpp
#include "camera_pins.h"
#include "sd_read_write.h"
```

Make sure these files are included in the sketch folder or are available through the correct library or example source.

---

## Recommended Arduino IDE Settings

| Setting | Value |
|---|---|
| Board package | ESP32 by Espressif Systems `2.0.17` |
| Board | ESP32S3 Dev Module or compatible ESP32-S3 camera board |
| PSRAM | Enabled |
| Upload speed | 921600 or lower if upload fails |
| Serial monitor baud | 115200 |
| Partition scheme | Huge APP or another large-app partition |

The exact board option may depend on the ESP32-S3 camera board being used.

---

## Pin Configuration

### Sensor and Output Pins

| Function | GPIO |
|---|---:|
| PIR sensor | GPIO 1 |
| IR LED | GPIO 2 |
| Light sensor | GPIO 46 |

### LoRa Radio Pins

| Function | GPIO |
|---|---:|
| NSS / CS | GPIO 3 |
| RESET | GPIO 14 |
| DIO0 | GPIO 20 |
| DIO1 | GPIO 21 |
| MISO | GPIO 47 |
| MOSI | GPIO 19 |
| SCK | GPIO 45 |

These pins are defined in the sketch:

```cpp
#define GPIO_PIR         1
#define GPIO_IRLED       2
#define GPIO_LIGHTSENSOR 46

#define PIN_NSS   3
#define PIN_RST   14
#define PIN_DIO0  20
#define PIN_DIO1  21

#define LORA_MISO  47
#define LORA_MOSI  19
#define LORA_SCK   45
```

Check that the pin definitions match the actual wiring of your ESP32-S3 board and LoRa module.

---

## Camera Configuration

The project is configured for:

```cpp
#define CAMERA_MODEL_ESP32S3_EYE
```

The camera captures grayscale QVGA images:

```cpp
.pixel_format = PIXFORMAT_GRAYSCALE,
.frame_size   = FRAMESIZE_QVGA,
```

This means the camera captures images at:

```text
320 x 240
```

The frame buffer is stored in PSRAM:

```cpp
.fb_location = CAMERA_FB_IN_PSRAM
```

Because of this, PSRAM must be enabled in the Arduino IDE board settings.

---

## Edge Impulse Model

The project uses an exported Edge Impulse model:

```cpp
#include <Wildsights_rhino_md_conf.5_inferencing.h>
```

The classifier is initialized in `setup()`:

```cpp
run_classifier_init();
```

Inference is run using:

```cpp
EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);
```

The code checks the Edge Impulse bounding box results for the label:

```text
rhino
```

The label comparison happens here:

```cpp
if (strcmp(bb.label, "rhino") == 0)
```

If your Edge Impulse model uses a different label name, update this line in the sketch.

---

## Detection Threshold

The rhino detection confidence threshold is currently set to `0.7`:

```cpp
if (rhino && best.value >= 0.7) {
```

Threshold behavior:

| Threshold | Behavior |
|---|---|
| `0.5` | More sensitive, but may create more false positives |
| `0.7` | Current default |
| `0.9` | More strict, but may miss some detections |

---

## LoRaWAN Configuration

The sketch uses OTAA activation with:

- Join EUI
- Device EUI
- App Key

These values are defined in the code:

```cpp
static const uint8_t JOIN_EUI_BE[8] = { ... };
static const uint8_t DEV_EUI_BE[8]  = { ... };
static const uint8_t APP_KEY[16]    = { ... };
```

Do not publish real LoRaWAN credentials in a public repository.

For public repositories, replace real credentials with placeholder values:

```cpp
static const uint8_t JOIN_EUI_BE[8] = {
  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00
};

static const uint8_t DEV_EUI_BE[8] = {
  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00
};

static const uint8_t APP_KEY[16] = {
  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00
};
```

If real keys were shared publicly, regenerate them in your LoRaWAN network server.

---

## LMIC Configuration

The LoRa radio uses custom SPI pins:

```cpp
SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, PIN_NSS);
```

LMIC is initialized in `setup()`:

```cpp
os_init();
LMIC_reset();
LMIC_selectSubBand(1);
LMIC_setAdrMode(0);
LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
LMIC_startJoining();
```

The selected LoRaWAN sub-band is:

```cpp
LMIC_selectSubBand(1);
```

Make sure this matches your LoRaWAN region, gateway, and network server configuration.

---

## LoRaWAN Uplink Messages

When a rhino is detected, the node sends:

```text
Rhino detected
```

When no rhino is detected, the node sends:

```text
No Rhino
```

Messages are sent on LoRaWAN port `1`:

```cpp
LMIC_setTxData2(1, buf, (u1_t)len, 0);
```

---

## SD Card Image Saving

Images are saved to:

```text
/camera/
```

The directory is created during setup:

```cpp
createDir(SD_MMC, "/camera");
```

Images are saved using an incrementing file number:

```text
/camera/1.jpg
/camera/2.jpg
/camera/3.jpg
```

Both rhino detections and non-detections are currently saved.

---

## How It Works

1. The ESP32-S3 starts the serial monitor, SD card, camera, Edge Impulse model, LoRa radio, and PIR interrupt.
2. The PIR sensor triggers an interrupt when motion is detected.
3. A FreeRTOS inference task wakes up.
4. The PIR signal is debounced.
5. The IR LED turns on.
6. The camera captures a grayscale image.
7. The image is resized to the Edge Impulse model input size.
8. The Edge Impulse classifier runs on the ESP32-S3.
9. The result is checked for a bounding box labeled `rhino`.
10. If the confidence is at least `0.7`, a rhino detection is confirmed.
11. The image is saved to the SD card.
12. A LoRaWAN message is queued.
13. The IR LED turns off.
14. The system waits through a cooldown period before responding to another trigger.

---

## Main Runtime Loop

The main `loop()` only runs the LMIC event loop:

```cpp
void loop() {
  os_runloop_once();
}
```

Image capture and inference are handled in a separate FreeRTOS task:

```cpp
xTaskCreatePinnedToCore(
  inferenceTask,
  "inferenceTask",
  32768,
  nullptr,
  1,
  &inferTaskHandle,
  1
);
```

---

## Serial Monitor

Open the Serial Monitor at:

```text
115200 baud
```

Startup messages include:

```text
PSRAM found: YES
Free heap:
Free PSRAM:
Camera PID:
Setup complete.
```

LoRaWAN messages include:

```text
Joining...
JOINED!
Uplink queued: Rhino detected
TX complete
```

Inference messages include:

```text
INFERENCE: running classifier
INFERENCE: done classifier
Rhino DETECTED
No rhino.
```

---

## Troubleshooting

### Camera Init Failed

Check the following:

- The correct camera model is selected
- `camera_pins.h` matches your board
- PSRAM is enabled
- ESP32 board package version is `2.0.17`
- The camera ribbon cable is connected correctly

### Failed to Allocate Snapshot Buffer

Check the following:

- PSRAM is enabled
- The board has PSRAM
- The Edge Impulse model is not too large
- The partition scheme provides enough app space

### Device Does Not Join LoRaWAN

Check the following:

- Join EUI
- Device EUI
- App Key
- EUI byte order
- LMIC region
- LMIC sub-band
- Gateway frequency plan
- LoRa radio wiring
- Antenna connection

### SD Card Images Are Not Saving

Check the following:

- SD card is inserted
- SD card is formatted correctly
- `sdmmcInit()` succeeds
- The `/camera` directory exists
- `sd_read_write.h` is included correctly

### Classifier Always Returns No Rhino

Check the following:

- The model label is exactly `rhino`
- The detection threshold is not too high
- The camera image orientation is correct
- The IR LED provides enough illumination
- The training data matches the deployment environment
- The model was exported correctly from Edge Impulse

---

## Notes

The function:

```cpp
saveGrayJpegWithBox()
```

saves the grayscale camera frame as a JPEG image.

In the current version of the sketch, the bounding box is printed to the Serial Monitor but is not drawn onto the saved image.

The bounding box output is printed here:

```cpp
Serial.printf(
  "Rhino DETECTED label = %s : conf=%.3f box(x=%u y=%u w=%u h=%u)\n",
  best.label,
  best.value,
  best.x,
  best.y,
  best.width,
  best.height
);
```

---

## Deployment Notes

- Use a weatherproof enclosure for outdoor deployment.
- Protect the camera lens, PIR sensor, and electronics from rain.
- Use a stable power supply.
- Connect the LoRa antenna before transmitting.
- Test the camera, SD card, Edge Impulse model, and LoRaWAN join process before field deployment.
- Do not publish real LoRaWAN credentials.

---

## License

Add the project license here.