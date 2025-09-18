#include <Wire.h>
#include <LSM303.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>

// Wi-Fi
#define WIFI_SSID "Pixel_5"
#define WIFI_PASSWORD "50035003"
WiFiClient wifiClient;


// MQTT Broker
const char* mqtt_server = "broker.hivemq.com"; // Replace with your MQTT broker's address
const int mqtt_port = 1883; // Default MQTT port
String baseTopicPath = "autoboa/";
PubSubClient mqttClient(wifiClient);

LSM303 compass;
LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};


char report[80];

void setup() {
  Serial.begin(9600);
  setup_wifi();
  mqttClient.setServer(mqtt_server, mqtt_port);
  ArduinoOTA.begin();

  Wire.begin(); 
  compass.init();
  compass.enableDefault();
}

void loop() {  
  // mqtt
  if (!mqttClient.connected()) {
    reconnect();
  }
  mqttClient.loop();

  compass.read();
  
  running_min.x = min(running_min.x, compass.m.x);
  running_min.y = min(running_min.y, compass.m.y);
  running_min.z = min(running_min.z, compass.m.z);

  running_max.x = max(running_max.x, compass.m.x);
  running_max.y = max(running_max.y, compass.m.y);
  running_max.z = max(running_max.z, compass.m.z);
  
  snprintf(report, sizeof(report), "min: {%+6d, %+6d, %+6d}    max: {%+6d, %+6d, %+6d}",
    running_min.x, running_min.y, running_min.z,
    running_max.x, running_max.y, running_max.z);
  Serial.println(report);
  publishTopic("CompassCalibration", report);

  ArduinoOTA.handle();
  delay(100);
}

// Connect to Wi-Fi
void setup_wifi() {
  delay(10);
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println("\nWi-Fi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void publishTopic(String topicName, String message) {
  if(mqttClient.connected()) {
    mqttClient.publish((baseTopicPath + topicName).c_str(), message.c_str());
  }
}

void reconnect() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);

    if (mqttClient.connect(clientId.c_str(), "", "", (baseTopicPath + "MqttState").c_str(), 0, true, "disconnected", false)) {
      Serial.println("connected");
      mqttClient.publish((baseTopicPath + "MqttState").c_str(), "connected", true);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" retrying in 5 seconds");
      delay(5000);
    }
  }
}
