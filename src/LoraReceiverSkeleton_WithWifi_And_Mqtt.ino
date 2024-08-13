#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <PubSubClient.h>

// WiFi credentials
const char* ssid = "Addie_IoT";
const char* password = "123";

// MQTT broker details
const char* mqttServer = "192.168.68.75";
const char* mqttUser = "mqttuser";
const char* mqttPassword = "mqttpassword";
const int mqttPort = 1883;

// MQTT client
WiFiClient espClient;
PubSubClient client(espClient);

// LoRa pins
#define ss 18
#define rst 14
#define dio0 26

// LED pin
#define ledPin 25 // You can change this to any available GPIO pin

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Receiver");

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Connect to MQTT broker
  client.setServer(mqttServer, mqttPort);
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect("LoRaReceiver", mqttUser, mqttPassword)) {
      Serial.println("Connected to MQTT");
    } else {
      delay(500);
    }
  }

  // Setup LoRa
  LoRa.setPins(ss, rst, dio0);
  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  // Set the same sync word as the sender
  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa Initializing OK!");

  // Setup LED pin
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW); // Ensure the LED is off initially
}

void loop() {
  client.loop();

  // Try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // Read packet
    String message = "";
    while (LoRa.available()) {
      message += (char)LoRa.read();
    }

    // Get RSSI value
    int rssi = LoRa.packetRssi();

    // Blink the LED to indicate a message was received
    digitalWrite(ledPin, HIGH);  // Turn the LED on
    delay(100);                  // Keep the LED on for 100 milliseconds
    digitalWrite(ledPin, LOW);   // Turn the LED off

    // Short delay before sending ACK
    delay(50);

    // Send acknowledgment
    LoRa.beginPacket();
    LoRa.print("ACK");
    LoRa.endPacket();
    Serial.println("ACK sent");

    // Send RSSI value to MQTT topic "mailbox/lora-rssi"
    String rssiStr = String(rssi);
    if (client.publish("mailbox/lora-rssi", rssiStr.c_str())) {
      Serial.println("RSSI value published successfully");
    } else {
      Serial.println("Failed to publish RSSI value");
    }

    // Determine topic based on message
    String topic;
    if (message.startsWith("CHECKIN")) {
      topic = "mailbox/lora-checkin";
    } else if (message.startsWith("EVENT")) {
      topic = "mailbox/lora-event";
    } else {
      topic = "mailbox/lora-default";
    }

    // Send message to MQTT broker
    if (client.publish(topic.c_str(), message.c_str())) {
      Serial.println("Message published successfully");
    } else {
      Serial.println("Failed to publish message");
    }

    Serial.print("Received message: ");
    Serial.println(message);
    Serial.print("RSSI: ");
    Serial.println(rssi);
  }
}
