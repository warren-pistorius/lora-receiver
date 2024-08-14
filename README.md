# LoRa Receiver with MQTT Integration and Time Synchronization

This project implements a LoRa-based receiver using the ESP32 microcontroller. The receiver listens for messages from a LoRa sender, synchronizes time, and publishes received data to an MQTT broker. The receiver also acknowledges received messages and responds to time requests from the sender.

## Features

- **LoRa Communication:** Receives data packets over LoRa at 915 MHz and sends acknowledgment (ACK) to the sender.
- **MQTT Integration:** Publishes received messages to an MQTT broker.
- **Time Synchronization:** Provides the current time to the sender upon request.
- **NTP Time Sync:** Synchronizes the receiver's time with an NTP server for accurate timekeeping.
- **Message Queue:** Queues messages if the MQTT connection is lost and publishes them once reconnected.

## Hardware Requirements

- **ESP32 Development Board**
- **LoRa Transceiver Module** (e.g., SX1276 or similar)
- **Optional:** OLED Display for visual output (e.g., Adafruit SSD1306)

## Pin Configuration

- **SS (Slave Select):** GPIO 18
- **RST (Reset):** GPIO 14
- **DIO0 (Interrupt):** GPIO 26
- **LED Indicator:** GPIO 25 (indicates system status)

## WiFi and MQTT Configuration

- **WiFi SSID:** `Addie_IoT`
- **WiFi Password:** `123`
- **MQTT Server:** `192.168.68.75`
- **MQTT User:** `mqttuser`
- **MQTT Password:** `mqttpassword`
- **MQTT Port:** `1883`

## Installation and Setup

1. **Clone the Repository:**
   ```bash
   git clone <repository-url>
   cd LoRa-Receiver
