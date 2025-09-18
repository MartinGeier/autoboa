#include <SPI.h>
#include <SD.h>
#include <Ticker.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <LSM303.h>
#include <ESP32Servo.h>

// Prepare the SW for use
// 1. Calibrate the compass using the Calibrate example from the LSM303 library
// 2. Calibrate the magnetometer using the Calibrate example from the LSM303 library
// 3. Update the magnetic declination value in the readCompass() function below to match the location the buoy will be used
// 4. Update the battery voltage correction value by measuring the actual battery voltage and 
//    comparing it to the indicated value of the ADC

#define SerialMon Serial

// Wi-Fi
#define WIFI_SSID "Pixel_5"
#define WIFI_PASSWORD "50035003"
WiFiClient wifiClient;


// MQTT Broker
const char* mqtt_server = "broker.hivemq.com"; // Replace with your MQTT broker's address
const int mqtt_port = 1883; // Default MQTT port
String baseTopicPath = "autoboa/";
PubSubClient mqttClient(wifiClient);


// GPS
int GPS_TX = 35;
int GPS_RX = 34;
SoftwareSerial SerialGPS(GPS_RX, GPS_TX);
TinyGPSPlus gps;


// magnetometer
LSM303 compass;
int SDAPIN = 21;
int SCLPIN = 22;


// Navigation
#define INITIAL_DISCARD_POINT_COUNT 60    // GPS points to discard after startup to avoid initial incorrect readings
#define TARGET_POINT_COUNT 120            // GPS points to use to get a stable target
#define RADIUS_OF_EARTH 6371000           // Earth's radius in meters
double lat,  lon;                         // working coordinates
double smoothedLat = 0.0;
double smoothedLon = 0.0;
double gpsAlpha = 0.05;            // Smoothing factor between 0 and 1; higher is less smooth but more responsive
double targetLat = 0.0;
double targetLon = 0.0;
double targetBearing = 0.0;        // current bearing to target
double targetDistance = 0.0;       // current distance to target
int pointCounter = 0;
int satelliteCount = 0;
int hdop = 0;


// compass
double heading = 0.0;              // current heading of bouy
double smoothedHeading = 0.0;      // smoothed heading
double headingAlpha = 0.2;         // smoothing factor for heading, smoothing factor between 0 and 1; higher is less smooth but more responsive


// Battery
#define BATT_ADC_PIN 33
#define R1 100000.0
#define R2 27000.0
#define BATT_VOLTAGE_CORRECTION 1.02375 // Correction factor for battery voltage


// station keeping
#define HOLDING_RADIUS 1.0         // Radius in meters for station-keeping
#define DEAD_ZONE_RADIUS 0.5       // Radius in meters to stop oscillation near target

// RGB LED pins
#define LED_RED_PIN 25
#define LED_GREEN_PIN 26
#define LED_BLUE_PIN 27


void setup()
{
  SerialMon.begin(9600);
  delay(2000);

  setup_wifi();

  mqttClient.setServer(mqtt_server, mqtt_port);

  Serial.println("Starting setup...");
  SerialGPS.begin(9600);

  ArduinoOTA.begin();

  // compass setup
  //Wire.begin();
  Wire.begin();//SDAPIN, SCLPIN);
  compass.init();
  compass.enableDefault();
  /*
  Calibration values; the default values of +/-32767 for each axis
  lead to an assumed magnetometer bias of 0. Use the Calibrate example
  program to determine appropriate values for your particular unit.
  */
  compass.m_min = (LSM303::vector<int16_t>){ -452, -502, -481};
  compass.m_max = (LSM303::vector<int16_t>) { +621, +616, +400};

  // Initialize RGB LED pins
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);
  
  // Set up battery ADC pin
  pinMode(BATT_ADC_PIN, INPUT);

  // Turn LED red initially to signal the bouy is acquiring GPS position
  setLedRed();

  Serial.println("End of setup");
}

void loop()
{
  // mqtt
  if (!mqttClient.connected()) {
    reconnect();
  }
  mqttClient.loop();

  // get GPS coordinates
  getGPSPosition(lat, lon);

  // read compass
  readCompass();

  // read battery
  readBatteryVoltage();

  // set led
  setLed();

  ArduinoOTA.handle();
  delay(10);
}

void setLed() {
  if(targetLat == 0) {
    setLedRed();    // set led to red indicating buoy does not have a target position
  } else if(targetDistance <= DEAD_ZONE_RADIUS) {
    setLedGreen();  // set led to green indicating buoy is at target position
  } else {
    setLedBlue();   // set led to blue indicating buoy is applying thrust to reach target position
  }
}

void getGPSPosition(double& lat, double& lon) {
  static unsigned long lastAcquiringPublish = 0;
  static bool acquired = false;

  // Process more GPS data per loop iteration
  unsigned long startTime = millis();
  while (SerialGPS.available() > 0 && (millis() - startTime) < 5) { // Process for up to 5ms
    char output = SerialGPS.read();
    gps.encode(output);

    if (gps.location.isUpdated()) {
      // ✅ Got a valid GPS fix
      lat = gps.location.lat();
      lon = gps.location.lng();
      satelliteCount = gps.satellites.value();
      hdop = gps.hdop.value();
      smoothCoordinates(lat, lon);
      aquireTargetPosition();
      targetBearing = calculateBearing(smoothedLat, smoothedLon, targetLat, targetLon);
      targetDistance = calculateDistance(smoothedLat, smoothedLon, targetLat, targetLon);
      writeGpsInfo();
      acquired = true;
      return;
    }
  }

  // Still acquiring → publish only every 5 seconds
  if (!acquired && millis() - lastAcquiringPublish >= 5000) {
    publishTopic("Log", "Acquiring GPS position...");
    lastAcquiringPublish = millis();
  }
}

void writeGpsInfo() {
  Serial.print("lat: "); 
  Serial.print(lat, 8);
  Serial.print(" | ");
  Serial.print("lon: "); 
  publishTopic("RawPosition", lat, lon);
  Serial.print(lon, 8);
  Serial.print(" | ");
  Serial.print("slat: "); 
  Serial.print(smoothedLat, 8);
  Serial.print(" | ");
  Serial.print("slon: "); 
  Serial.print(smoothedLon, 8);
  publishTopic("SmoothedPosition", smoothedLat, smoothedLon);
  Serial.print("   |   ");
  Serial.print("tlat: "); 
  Serial.print(targetLat, 8);
  Serial.print(" | ");
  Serial.print("tlon: "); 
  Serial.print(targetLon, 8);
  publishTopic("TargetPosition", targetLat, targetLon);
  Serial.print("   |   ");
  Serial.print("dtt: "); 
  Serial.print(targetDistance, 2);
  publishTopic("TargetDistance", targetDistance);
  Serial.print("   |   ");
  Serial.print("btt: "); 
  Serial.print(targetBearing, 0);
  publishTopic("TargetBearing", targetBearing);
  Serial.print("   |   ");
  Serial.print("sat: "); 
  Serial.print(satelliteCount, 0);
  publishTopic("SatelliteCount", satelliteCount);
  Serial.print("   |   ");
  Serial.print("hdop: "); 
  Serial.print(hdop, 0);
  publishTopic("HDOP", hdop);
  Serial.println();
}

void aquireTargetPosition() {
  if (pointCounter > INITIAL_DISCARD_POINT_COUNT && pointCounter <= TARGET_POINT_COUNT) {
    targetLat = smoothedLat;
    targetLon = smoothedLon;
  }
  
  if (pointCounter <= TARGET_POINT_COUNT){
    pointCounter++;
  }
}

// Exponential Smoothing of gps coordinates
void smoothCoordinates(double newLat, double newLon) {
  if(pointCounter < INITIAL_DISCARD_POINT_COUNT) {
    // avoid smooting until we have a stable position
    smoothedLat = newLat;
    smoothedLon = newLon;
  }
  else {
    smoothedLat = gpsAlpha * newLat + (1 - gpsAlpha) * smoothedLat;
    smoothedLon = gpsAlpha * newLon + (1 - gpsAlpha) * smoothedLon;
  }
}

// Calculates the bearing from (lat, lon) to (targetLat, targetLon) assuming a flat Earth
double calculateBearing(double lat, double lon, double targetLat, double targetLon) {
  if(pointCounter <= TARGET_POINT_COUNT) {
    return -1;
  }

  double deltaLon = (targetLon - lon) * 111320 * cos(toRadians(lat));  // Adjust for latitude
  double deltaLat = (targetLat - lat) * 111320;

  double bearing = atan2(deltaLon, deltaLat);  // atan2 returns radians  
  return fmod((toDegrees(bearing) + 360.0), 360.0);      // Normalize to 0-360°
}

// Converts degrees to radians
double toRadians(double degrees) {
  return degrees * PI / 180.0;
}

// Converts radians to degrees
double toDegrees(double radians) {
  return radians * 180.0 / PI;    
}

// Calculates the distance between two points
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  if(pointCounter <= TARGET_POINT_COUNT) {
    return 0;
  }

  lat1 = toRadians(lat1);
  lon1 = toRadians(lon1);
  lat2 = toRadians(lat2);
  lon2 = toRadians(lon2);

  double dLat = lat2 - lat1;
  double dLon = lon2 - lon1;

  double a = sin(dLat / 2) * sin(dLat / 2) +
            cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));

  return RADIUS_OF_EARTH * c;
}

void readCompass() {
  static unsigned long lastCompassRead = 0;  // timing for compass readings

  if (millis() - lastCompassRead < 1000) {
    return; // read compass every second
  }

  Serial.println("Starting to read compass...");
  compass.read();
  Serial.println("...compass read!");

  // Tilt compensation
  double roll = atan2(compass.a.y, compass.a.z);
  double pitch = atan2(-compass.a.x, sqrt(pow(compass.a.y, 2) + pow(compass.a.z, 2)));

  double mag_x = compass.m.x;
  double mag_y = compass.m.y;
  double mag_z = compass.m.z;

  // Apply hard-iron correction
  mag_x -= (compass.m_min.x + compass.m_max.x) / 2.0;
  mag_y -= (compass.m_min.y + compass.m_max.y) / 2.0;
  mag_z -= (compass.m_min.z + compass.m_max.z) / 2.0;

  // Apply soft-iron correction
  double avg_delta_x = (compass.m_max.x - compass.m_min.x) / 2.0;
  double avg_delta_y = (compass.m_max.y - compass.m_min.y) / 2.0;
  double avg_delta_z = (compass.m_max.z - compass.m_min.z) / 2.0;
  double avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3.0;

  if (avg_delta_x != 0) mag_x *= avg_delta / avg_delta_x;
  if (avg_delta_y != 0) mag_y *= avg_delta / avg_delta_y;
  if (avg_delta_z != 0) mag_z *= avg_delta / avg_delta_z;

  // Tilt-compensated magnetometer values
  double x_h = mag_x * cos(pitch) + mag_y * sin(pitch) * sin(roll) - mag_z * sin(pitch) * cos(roll);
  double y_h = mag_y * cos(roll) + mag_z * sin(roll);
  
  double headingRad = atan2(y_h, x_h);

  // Convert to degrees and apply declination
  heading = toDegrees(headingRad);
  heading += 3.5; //magnetic declination for northern Italy
  if (heading < 0) heading += 360;
  if (heading >= 360) heading -= 360;
  
  lastCompassRead = millis();

  // Vector-based smoothing for heading (handles wrap-around)
  static double smoothedX = 0.0;
  static double smoothedY = 0.0;
  headingRad = toRadians(heading);
  double x = cos(headingRad);
  double y = sin(headingRad);

  if (smoothedX == 0.0 && smoothedY == 0.0) {
    smoothedX = x;
    smoothedY = y;
  } else {
    smoothedX = headingAlpha * x + (1 - headingAlpha) * smoothedX;
    smoothedY = headingAlpha * y + (1 - headingAlpha) * smoothedY;
  }

  smoothedHeading = fmod((toDegrees(atan2(smoothedY, smoothedX)) + 360.0), 360.0);

  publishTopic("RawHeading", heading);
  publishTopic("SmoothedHeading", smoothedHeading);
}

void readBatteryVoltage() {
  static unsigned long lastBatteryRead = 0;
  if (millis() - lastBatteryRead < 5000) { // Read every 5 seconds
    return;
  }
  lastBatteryRead = millis();

  int adcValue = analogRead(BATT_ADC_PIN);
  // ESP32 ADC is 12-bit (0-4095) and reference voltage is 3.3V
  // An internal attenuation is set by default, giving a 0-3.3V range
  double vOut = adcValue * (3.3 / 4095.0);
  double vIn = vOut * (R1 + R2) / R2;
  vIn *= BATT_VOLTAGE_CORRECTION;

  publishTopic("BatteryVoltage", vIn);
  Serial.print("Battery Voltage: ");
  Serial.println(vIn, 2);
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

// Reconnect to MQTT broker
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

void publishTopic(String topicName, double lat, double lon) {
  publishTopic(topicName, String(lat, 8) + ", " + String(lon, 8));
}

void publishTopic(String topicName, double value) {
  publishTopic(topicName, String(value, 8));
}

void publishTopic(String topicName, String message) {
  if(mqttClient.connected()) {
    mqttClient.publish((baseTopicPath + topicName).c_str(), message.c_str());
  }
}

// Set RGB LED colors
void setRGBLed(bool red, bool green, bool blue) {
  digitalWrite(LED_RED_PIN, red ? HIGH : LOW);
  digitalWrite(LED_GREEN_PIN, green ? HIGH : LOW);
  digitalWrite(LED_BLUE_PIN, blue ? HIGH : LOW);
}

// Convenience methods for single colors
void setLedRed() {
  setRGBLed(true, false, false);
}

void setLedGreen() {
  setRGBLed(false, true, false);
}

void setLedBlue() {
  setRGBLed(false, false, true);
}

void setLedOff() {
  setRGBLed(false, false, false);
}