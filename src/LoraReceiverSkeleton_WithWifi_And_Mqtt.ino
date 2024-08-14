#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <time.h>
#include <TimeLib.h>

// WiFi credentials
const char* ssid = "Addie_IoT";
const char* password = "123";

// MQTT broker details
const char* mqttServer = "192.168.68.75";
const char* mqttUser = "mqttuser";
const char* mqttPassword = "mqttpassword";
const int mqttPort = 1883;

//NTP setup
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 36000;  // Melbourne is UTC+10 (10 hours * 3600 seconds)
const int   daylightOffset_sec = 0;  // 1 hour daylight saving time

// MQTT client
WiFiClient espClient;
PubSubClient client(espClient);

// LoRa pins
#define ss 18
#define rst 14
#define dio0 26

// LED pin
#define ledPin 25

// Connection check intervals
const unsigned long wifiCheckInterval = 30000;  // 30 seconds
const unsigned long mqttCheckInterval = 60000;  // 60 seconds

unsigned long lastWifiCheck = 0;
unsigned long lastMqttCheck = 0;

unsigned long lastNTPSync = 0;
const unsigned long ntpSyncInterval = 24 * 60 * 60 * 1000;  // 24 hours

struct tm timeinfo;

// Message queue
#define MAX_QUEUE_SIZE 20
struct QueuedMessage {
  String topic;
  String message;
};
QueuedMessage messageQueue[MAX_QUEUE_SIZE];
int queueFront = 0;
int queueRear = -1;
int queueSize = 0;

bool serialAvailable = false;


void checkAndUpdateNTPTime() {
  if (millis() - lastNTPSync >= ntpSyncInterval) {
    setupTime();
    lastNTPSync = millis();
  }
}

void setup() {
  Serial.begin(115200);
  unsigned long startTime = millis();
  while (!Serial && millis() - startTime < 5000);
  serialAvailable = Serial;  // This will be true if Serial is available, false otherwise
  
  if (serialAvailable) {
    Serial.println("LoRa Receiver");
  }

  pinMode(ledPin, OUTPUT);

  setupLoRa();
  setupWifi();
  setupTime(); 
  setupMqtt();
}

void loop() {
  unsigned long currentMillis = millis();

  // Check WiFi connection
  if (currentMillis - lastWifiCheck >= wifiCheckInterval) {
    checkWifiConnection();
    lastWifiCheck = currentMillis;
  }

  // Check MQTT connection
  if (currentMillis - lastMqttCheck >= mqttCheckInterval) {
    checkMqttConnection();
    lastMqttCheck = currentMillis;
  }

  client.loop();

  checkAndUpdateNTPTime();

  // Process queued messages
  processQueue();

  // LoRa receiving logic
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    handleLoRaMessage();
  }
}

void setupWifi() {
  if (serialAvailable) {
    Serial.println("Connecting to WiFi...");
  }
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    if (serialAvailable) {
      Serial.print(".");
    }
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    if (serialAvailable) {
      Serial.println("\nConnected to WiFi");
      digitalWrite(ledPin, HIGH);
    }
  } else {
    if (serialAvailable) {
      Serial.println("\nFailed to connect to WiFi. Will retry later.");
      Serial.println(WiFi.status());
    }
    digitalWrite(ledPin, LOW);
  }
}

void checkWifiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    if (serialAvailable) {
      Serial.println("WiFi connection lost. Reconnecting...");
    }
    WiFi.disconnect();
    setupWifi();
  }
}

void setupMqtt() {
  client.setServer(mqttServer, mqttPort);
  connectMqtt();
}

void connectMqtt() {
  int attempts = 0;
  while (!client.connected() && attempts < 3) {
    if (serialAvailable) {
      Serial.println("Connecting to MQTT...");
    }
    if (client.connect("LoRaReceiver", mqttUser, mqttPassword)) {
      if (serialAvailable) {
        Serial.println("Connected to MQTT");
        digitalWrite(ledPin, HIGH);
      }
    } else {
      if (serialAvailable) {
        Serial.print("Failed to connect to MQTT, rc=");
        Serial.print(client.state());
        Serial.println(" Retrying in 5 seconds");
        digitalWrite(ledPin, LOW);
      }
      delay(5000);
      attempts++;
    }
  }
}

void checkMqttConnection() {
  if (!client.connected()) {
    if (serialAvailable) {
      Serial.println("MQTT connection lost. Reconnecting...");
    }
    connectMqtt();
  }
}

void setupLoRa() {
  LoRa.setPins(ss, rst, dio0);
  LoRa.setTxPower(19);
  
  if (!LoRa.begin(915E6)) {
    if (serialAvailable) {
      Serial.println("Starting LoRa failed!");
    }
    while (1);
  }

  LoRa.setSyncWord(0xF3);
  if (serialAvailable) {
    Serial.println("LoRa Initializing OK!");
  }
}

void handleLoRaMessage() {
  String message = "";
  while (LoRa.available()) {
    message += (char)LoRa.read();
  }

  int rssi = LoRa.packetRssi();

  digitalWrite(ledPin, HIGH);
  delay(100);
  digitalWrite(ledPin, LOW);
  delay(100);
  digitalWrite(ledPin, HIGH);

  delay(50);

  LoRa.beginPacket();
  LoRa.print("ACK");
  LoRa.endPacket();
  if (serialAvailable) {
    Serial.println("ACK sent");
  }

  publishToMqtt("mailbox/lora-rssi", String(rssi));

  String topic = "mailbox/lora-default";

  if (message == "TIME_REQUEST") {
    time_t now;
    
    if (getLocalTime(&timeinfo)) {
      char timeStr[25];
      strftime(timeStr, sizeof(timeStr), "TIME %Y-%m-%d %H:%M:%S", &timeinfo);
      LoRa.beginPacket();
      LoRa.print(timeStr);
      LoRa.endPacket();
      if (serialAvailable) {
        Serial.println("Time sent: " + String(timeStr));
      }
    }
  }
  else if (message.startsWith("CHECKIN")) {
    topic = "mailbox/lora-checkin";
  } else if (message.startsWith("EVENT")) {
    topic = "mailbox/lora-event";
  }

  publishToMqtt(topic.c_str(), message);

  if (serialAvailable) {
    Serial.print("Received message: ");
    Serial.println(message);
    Serial.print("RSSI: ");
    Serial.println(rssi);
  }
}

void publishToMqtt(const char* topic, const String& message) {
  if (client.connected()) {
    if (client.publish(topic, message.c_str())) {
      if (serialAvailable) {
        Serial.println("Message published successfully");
      }
    } else {
      if (serialAvailable) {
        Serial.println("Failed to publish message, queueing...");
      }
      enqueueMessage(topic, message);
    }
  } else {
    if (serialAvailable) {
      Serial.println("MQTT not connected. Queueing message.");
    }
    enqueueMessage(topic, message);
  }
}

void enqueueMessage(const String& topic, const String& message) {
  if (queueSize < MAX_QUEUE_SIZE) {
    queueRear = (queueRear + 1) % MAX_QUEUE_SIZE;
    messageQueue[queueRear].topic = topic;
    messageQueue[queueRear].message = message;
    queueSize++;
    if (serialAvailable) {
      Serial.println("Message queued successfully");
    }
  } else {
    if (serialAvailable) {
      Serial.println("Queue is full, message discarded");
    }
  }
}

void processQueue() {
  if (queueSize > 0 && client.connected()) {
    if (client.publish(messageQueue[queueFront].topic.c_str(), messageQueue[queueFront].message.c_str())) {
      if (serialAvailable) {
        Serial.println("Queued message published successfully");
      }
      dequeueMessage();
    } else {
      if (serialAvailable) {
        Serial.println("Failed to publish queued message, will retry later");
      }
    }
  }
}

void dequeueMessage() {
  queueFront = (queueFront + 1) % MAX_QUEUE_SIZE;
  queueSize--;
}

void setupTime() {
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  
  int retries = 0;
  const int maxRetries = 5;
  while (!getLocalTime(&timeinfo) && retries < maxRetries) {
    if (serialAvailable) {
      Serial.println("Failed to obtain time, retrying...");
    }
    delay(1000);
    retries++;
  }
  
  if (retries == maxRetries) {
    if (serialAvailable) {
      Serial.println("Failed to synchronize time after multiple attempts");
    }
    return;
  }
  
  setTime(timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year + 1900);
  
  if (serialAvailable) {
    char timeStr[30];
    snprintf(timeStr, sizeof(timeStr), "%04d-%02d-%02d %02d:%02d:%02d", 
             timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, 
             timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    Serial.print("Time synchronized with NTP server: ");
    Serial.println(timeStr);
  }
}
