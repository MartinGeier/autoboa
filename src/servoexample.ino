/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald

 modified for the ESP32 on March 2017
 by John Bennett

 see http://www.arduino.cc/en/Tutorial/Sweep for a description of the original code

 * Different servos require different pulse widths to vary servo angle, but the range is 
 * an approximately 500-2500 microsecond pulse every 20ms (50Hz). In general, hobbyist servos
 * sweep 180 degrees, so the lowest number in the published range for a particular servo
 * represents an angle of 0 degrees, the middle of the range represents 90 degrees, and the top
 * of the range represents 180 degrees. So for example, if the range is 1000us to 2000us,
 * 1000us would equal an angle of 0, 1500us would equal 90 degrees, and 2000us would equal 1800
 * degrees.
 * 
 * Circuit: (using an ESP32 Thing from Sparkfun)
 * Servo motors have three wires: power, ground, and signal. The power wire is typically red,
 * the ground wire is typically black or brown, and the signal wire is typically yellow,
 * orange or white. Since the ESP32 can supply limited current at only 3.3V, and servos draw
 * considerable power, we will connect servo power to the VBat pin of the ESP32 (located
 * near the USB connector). THIS IS ONLY APPROPRIATE FOR SMALL SERVOS. 
 * 
 * We could also connect servo power to a separate external
 * power source (as long as we connect all of the grounds (ESP32, servo, and external power).
 * In this example, we just connect ESP32 ground to servo ground. The servo signal pins
 * connect to any available GPIO pins on the ESP32 (in this example, we use pin 18.
 * 
 * In this example, we assume a Tower Pro MG995 large servo connected to an external power source.
 * The published min and max for this servo is 1000 and 2000, respectively, so the defaults are fine.
 * These values actually drive the servos a little past 0 and 180, so
 * if you are particular, adjust the min and max values to match your needs.
 */

#include <ESP32Servo.h>
#include <Wire.h>
#include <LSM303.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

Servo myservo;  // create servo object to control a servo
// 16 servo objects can be created on the ESP32

int pos = 0;    // variable to store the servo position
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
// Possible PWM GPIO pins on the ESP32-S2: 0(used by on-board button),1-17,18(used by on-board LED),19-21,26,33-42
// Possible PWM GPIO pins on the ESP32-S3: 0(used by on-board button),1-21,35-45,47,48(used by on-board LED)
// Possible PWM GPIO pins on the ESP32-C3: 0(used by on-board button),1-7,8(used by on-board LED),9-10,18-21
// #if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)
// int servoPin = 17;
// #elif defined(CONFIG_IDF_TARGET_ESP32C3)
// int servoPin = 7;
// #else
// int servoPin = 18;
// #endif
int MOTORPIN = 12;

// magnetometer
LSM303 compass;
int SDAPIN = 21;
int SCLPIN = 22;

// GPS
int GPS_TX = 35;
int GPS_RX = 34;
SoftwareSerial ss(GPS_RX, GPS_TX);
TinyGPSPlus gps;

void setup() {
  Serial.begin(9600);

  // motor control setup
	// Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	myservo.setPeriodHertz(50);    // standard 50 hz servo
	myservo.attach(MOTORPIN, 1000, 2000); // attaches the servo on pin 18 to the servo object
	// using default min/max of 1000us and 2000us
	// different servos may require different min/max settings
	// for an accurate 0 to 180 sweep

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
  compass.m_min = (LSM303::vector<int16_t>){-422, -427, -405};
  compass.m_max = (LSM303::vector<int16_t>){+545, +562, +437};

  ss.begin(9600);

  delay(2000);
  Serial.println("Setup done");
}

void loop() {
  // myservo.write(90); // we need to write 90 to start the ESC. 
  // // 90=motor off, >90 = reverse, < 90=forward
  // // a ramp up/down is necessary to avoid too much force on the motor
  // delay(1000);
  // myservo.write(80);
  // delay(1000);
  // myservo.write(50);
  // delay(1000);
  // myservo.write(0);
  // delay(1000);
  // myservo.write(60);
  // delay(1000);
  // myservo.write(80);
  // delay(1000);

  compass.read();
  
  /*
  When given no arguments, the heading() function returns the angular
  difference in the horizontal plane between a default vector and
  north, in degrees.
  
  The default vector is chosen by the library to point along the
  surface of the PCB, in the direction of the top of the text on the
  silkscreen. This is the +X axis on the Pololu LSM303D carrier and
  the -Y axis on the Pololu LSM303DLHC, LSM303DLM, and LSM303DLH
  carriers.
  
  To use a different vector as a reference, use the version of heading()
  that takes a vector argument; for example, use
  
    compass.heading((LSM303::vector<int>){0, 0, 1});
  
  to use the +Z axis as a reference.
  */
  float heading = compass.heading((LSM303::vector<int>){1, 0, 0});
  
  //Serial.println("Heading:" + String(heading));

  // read GPS coordinates
  while (ss.available() > 0){
    gps.encode(ss.read());
    if (gps.location.isUpdated()){
      Serial.print(gps.location.lat(), 6);
      Serial.print(", "); 
      Serial.println(gps.location.lng(), 6);
    }
  }

  delay(100);

}