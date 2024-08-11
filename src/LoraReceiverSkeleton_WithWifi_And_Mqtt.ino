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
  Serial.println("LoRa Initializing OK!");
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
  }
}