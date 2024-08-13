#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// WiFi credentials
const char* ssid = "Addie_IoT";
const char* password = "IOTW@llrus83";

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

// OLED display settings
#define OLED_SDA 4
#define OLED_SCL 15 
#define OLED_RST 16
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

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

  // Initialize OLED display
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0,0);      // Start at top-left corner
  display.display();           // Display buffer
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

    // Display the received message and RSSI on the OLED screen
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Received message:");
    display.println(message);
    display.print("RSSI: ");
    display.println(rssi);
    display.display(); // Show the message and RSSI on the screen

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
