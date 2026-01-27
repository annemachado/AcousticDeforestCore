# AcousticDeforestCore

## What is it?
AcousticDeforestCore is an Arduino library that packages the core firmware modules used by an ESP32 acoustic deforestation prototype. It provides a small protocol framing layer, a LoRa link with ACK metrics, and helpers to integrate the VoiceRecognitionV3_ESP32 (VR3) module, plus example sketches for bench and E2E validation.

## Requirements
- **Hardware:** ESP32 + SX127x LoRa radio + VR3 (VoiceRecognitionV3_ESP32) module.
- **Arduino library dependencies:**
  - **LoRa** (Sandeep Mistry) — required by `lora_link`.
  - **VoiceRecognitionV3_ESP32** — required by `vr_link`.

## Installation (Arduino IDE)
1. Copy the `AcousticDeforestCore` folder into your Arduino libraries directory, e.g. `Documents/Arduino/libraries/AcousticDeforestCore`.
2. In **Library Manager**, install:
   - **LoRa** by Sandeep Mistry.
   - **VoiceRecognitionV3_ESP32** [annemachado/VoiceRecognitionV3_ESP32](https://github.com/annemachado/VoiceRecognitionV3_ESP32) (install it manually if it is not listed in the manager).
3. Open any example sketch from **File > Examples > AcousticDeforestCore**.

## Project structure
```
AcousticDeforestCore/
  LICENSE
  README.md
  library.properties
  keywords.txt
  AcousticDeforestCore.h
  protocol.h / protocol.cpp
  lora_link.h / lora_link.cpp
  vr_link.h / vr_link.cpp
  event_model.h
  logger.h
  examples/
    base_ack/base_ack.ino
    sensor_tx/sensor_tx.ino
    e2e_bench_sensor/e2e_bench_sensor.ino
```

## How to run each example
- **base_ack**
  - Purpose: Base station that receives STATUS/ALERT packets and replies with ACK.
  - Upload `examples/base_ack/base_ack.ino` to the base ESP32 + LoRa radio.
  - Open Serial Monitor at **115200** and wait for incoming packets.
  - Optional commands: `sf <7..12>`, `status`, `help`.

- **sensor_tx**
  - Purpose: Bench transmitter that emits STATUS/ALERT payloads and logs ACK metrics.
  - Upload `examples/sensor_tx/sensor_tx.ino` to the sensor ESP32 + LoRa radio.
  - Configure via Serial Monitor and issue `start` to begin sending.
  - Key commands: `run`, `dist`, `n`, `mode`, `period`, `sf`, `start`, `stop`.

- **e2e_bench_sensor**
  - Purpose: End-to-end demo combining VR3 recognition with LoRa alerts.
  - Upload `examples/e2e_bench_sensor/e2e_bench_sensor.ino` to ESP32 + LoRa + VR3.
  - Use `load` / `loadlist` to preload VR records, then `run` to emit alerts.

## API overview
- **protocol** (`protocol.h/.cpp`)
  - Packs/unpacks 11B STATUS/ALERT frames and 5B ACK frames.
- **lora_link** (`lora_link.h/.cpp`)
  - Initializes LoRa parameters, sends STATUS/ALERT with ACK retries, and parses received packets.
- **vr_link** (`vr_link.h/.cpp`)
  - Wraps VoiceRecognitionV3_ESP32 driver, supports record loading, training, and polling.
- **event_model** (`event_model.h`)
  - Minimal struct for detection events.
- **logger** (`logger.h`)
  - Lightweight Serial logging macros.

## Protocol contracts (11B / 5B, little-endian)
- **STATUS/ALERT frame (11 bytes)**
  - `dev_id` (1B)
  - `msg_type` (1B) — `MSG_STATUS` or `MSG_ALERT`
  - `uptime_ms` (4B, little-endian)
  - `seq` (2B, little-endian)
  - `field8` (1B) — Flags (STATUS) or EventClass (ALERT)
  - `battery_mv` (2B, little-endian)

- **ACK frame (5 bytes)**
  - `msg_type` (1B) — `MSG_ACK`
  - `seq` (2B, little-endian)
  - `rssi_dbm` (1B, signed)
  - `snr_db` (1B, signed)

## License
MIT License — see [LICENSE](LICENSE).
