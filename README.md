# WildSight — Senior Design Project (Computer Vision + LoRaWAN “Track & Trace”)

RhinoGuard is a senior design project focused on **detecting and tracking rhinos using computer vision** and relaying lightweight event data over **LoRaWAN** to a cloud dashboard. The goal is a field-deployable system that can help conservation teams **monitor animal presence and movement**, support **early warning** workflows, and enable **traceability** of sightings across time and location.

> This repository contains the embedded LoRaWAN transmitter portion of the system (ESP32 + RFM9x), along with notes on how it fits into the end-to-end pipeline.

---

## Project Overview

### Problem
Wildlife conservation teams often need low-power, long-range telemetry for remote environments where cellular connectivity is unreliable or expensive. In addition, camera-based systems can generate high bandwidth data that is not feasible to transmit continuously from the field.

### Approach
RhinoGuard combines:
- **Computer Vision at the edge** to detect rhinos in camera frames.
- **Event-based reporting**: transmit only small messages such as “rhino detected” with optional metadata (timestamp, confidence, camera ID).
- **LoRaWAN connectivity** using a gateway + The Things Stack (The Things Industries) to forward data to the cloud.

---

## System Architecture

1. **Edge Vision Node**
   - Captures frames from a camera.
   - Runs a CV model to detect rhinos (and optionally identify or classify).
   - Produces compact “events” rather than streaming video.

2. **LoRaWAN Telemetry Node (this repo)**
   - Microcontroller: **ESP32-S3**
   - Radio: **Adafruit RFM9x 915 MHz**
   - Sends OTAA LoRaWAN uplinks containing a payload string (ASCII bytes).

---

## Repository Contents

- `LoRa-Sending.ino`  
  LoRaWAN OTAA uplink sender using MCCI LMIC. Payload is a configurable string (e.g., `"Hello World"`).

---

## Hardware

- **Freenove ESP32-S3-WROOM-1**
- **Adafruit RFM9x (915 MHz)**
- **RAK7268V2 Gateway**
- Region: **US915**

### Example Wiring for LoRa (typical)
This project uses the following pin definitions in the sketch (edit as needed for your wiring):

- NSS / CS: `PIN_NSS = 3`
- RESET: `PIN_RST = 5`
- DIO0: `PIN_DIO0 = 4`
- DIO1: `PIN_DIO1 = 6`

---

## Software Requirements

- Arduino IDE
- **MCCI LoRaWAN LMIC** library (install via Arduino Library Manager)

---

## The Things Stack (TTI) Setup

### Create an Application
In The Things Stack:
1. Create an application for the project (e.g., `rhinoguard`).

### Register an End Device (OTAA)
1. Register a new end device with OTAA activation.
2. Copy the generated:
   - **JoinEUI** (AppEUI)
   - **DevEUI**
   - **AppKey**
3. Paste them into the sketch arrays:
   - `JOIN_EUI_BE[8]`
   - `DEV_EUI_BE[8]`
   - `APP_KEY[16]`

> In this code, JoinEUI and DevEUI are stored in big-endian, then reversed in `os_getArtEui()` / `os_getDevEui()` for LMIC.

---

## Gateway Setup Notes (US915 / Sub-bands)

US915 uses channel plans and (often) “sub-bands” (FSB1/FSB2/etc.) depending on gateway configuration.  
This sketch selects:
```cpp
LMIC_selectSubBand(1);
