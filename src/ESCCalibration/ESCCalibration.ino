 /*
 * ESC Calibration Sketch
 * 
 * This sketch calibrates ESCs by sending the full throttle range.
 * 
 * SAFETY: REMOVE PROPELLERS BEFORE RUNNING THIS!
 * 
 * Instructions:
 * 1. Remove propellers from motors
 * 2. Connect ESP32 to computer via USB
 * 3. Connect ESC signal wires to pins 18 and 19
 * 4. Keep ESC power OFF
 * 5. Upload this sketch
 * 6. Open Serial Monitor (9600 baud)
 * 7. When prompted, power ON the ESCs
 * 8. Follow the prompts in Serial Monitor
 * 9. ESCs will beep to confirm each step
 * 10. When complete, upload your normal buoy.ino sketch
 */

 // To calibrate unglug the internal power plug to the ESC's
 // Upload the program via USB powering the ESP32 only via USB
 // When instructed turn on the power switch and plug the power plug back in

#include <ESP32Servo.h>

// Motor control pins (same as main sketch)
#define LEFT_MOTOR_PIN 18
#define RIGHT_MOTOR_PIN 19

// Motor Objects
Servo leftMotor;
Servo rightMotor;

void setup() {
  Serial.begin(9600);
  delay(2000);
  
  // Serial.println("========================================");
  // Serial.println("    ESC CALIBRATION TOOL");
  // Serial.println("========================================");
  // Serial.println();
  // Serial.println("WARNING: MAKE SURE PROPELLERS ARE REMOVED!");
  // Serial.println();
  // Serial.println("This will calibrate both ESCs simultaneously.");
  // Serial.println();
  
  // // Attach motors
  Serial.println("Attaching motors to pins 18 and 19...");
  leftMotor.attach(LEFT_MOTOR_PIN);
  rightMotor.attach(RIGHT_MOTOR_PIN);
  delay(500);
  
  // // Step 1: Send FULL THROTTLE
  // Serial.println();
  // Serial.println("Step 1: Sending FULL THROTTLE (2000us)");
  // Serial.println(">>> POWER ON THE ESCs NOW <<<");
  // Serial.println("ESCs should beep to recognize full throttle...");
  // leftMotor.writeMicroseconds(2000);
  // rightMotor.writeMicroseconds(2000);
  // delay(10000);  // Wait 5 seconds
  
  // // Step 2: Send FULL REVERSE
  // Serial.println();
  // Serial.println("Step 2: Sending FULL REVERSE (1000us)");
  // Serial.println("ESCs should beep to recognize full reverse...");
  // leftMotor.writeMicroseconds(1000);
  // rightMotor.writeMicroseconds(1000);
  // delay(5000);  // Wait 5 seconds
  
  // // Step 3: Send NEUTRAL
  // Serial.println();
  // Serial.println("Step 3: Sending NEUTRAL (1500us)");
  // Serial.println("ESCs should beep to confirm calibration is stored...");
  leftMotor.writeMicroseconds(1500);
  rightMotor.writeMicroseconds(1500);
  // delay(3000);
  
  // // Complete
  // Serial.println();
  // Serial.println("========================================");
  // Serial.println("    CALIBRATION COMPLETE!");
  // Serial.println("========================================");
  // Serial.println();
  
  // // Test motors
  // Serial.println("Now testing motors at low speed...");
  // Serial.println("WARNING: Motors will spin briefly!");
  delay(2000);
  

  
  // Serial.println();
  // Serial.println("Your ESCs are now calibrated and tested.");
  // Serial.println("You can now upload your normal buoy.ino sketch.");
  // Serial.println();
  // Serial.println("The ESCs will stay at neutral (1500us).");
  // Serial.println("You can power off when ready.");
  // Serial.println();
}

void loop() {
  // Keep sending neutral signal
  testMotors();
  delay(500);
}

void testMotors() {
  Serial.println();
  Serial.println("Testing LEFT motor forward...");
  leftMotor.writeMicroseconds(1700);  // Slow forward
  delay(1000);
  leftMotor.writeMicroseconds(1500);  // Stop
  delay(500);
  
  Serial.println("Testing LEFT motor reverse...");
  leftMotor.writeMicroseconds(1300);  // Slow reverse
  delay(1000);
  leftMotor.writeMicroseconds(1500);  // Stop
  delay(1000);
  
  Serial.println("Testing RIGHT motor forward...");
  rightMotor.writeMicroseconds(1700);  // Slow forward
  delay(1000);
  rightMotor.writeMicroseconds(1500);  // Stop
  delay(500);
  
  Serial.println("Testing RIGHT motor reverse...");
  rightMotor.writeMicroseconds(1300);  // Slow reverse
  delay(1000);
  rightMotor.writeMicroseconds(1500);  // Stop
  delay(1000);
  
  Serial.println("Testing BOTH motors forward...");
  leftMotor.writeMicroseconds(1700);
  rightMotor.writeMicroseconds(1700);
  delay(1000);
  leftMotor.writeMicroseconds(1500);
  rightMotor.writeMicroseconds(1500);
  delay(500);
  
  Serial.println("Testing BOTH motors reverse...");
  leftMotor.writeMicroseconds(1300);
  rightMotor.writeMicroseconds(1300);
  delay(1000);
  leftMotor.writeMicroseconds(1500);
  rightMotor.writeMicroseconds(1500);
  
  Serial.println();
  Serial.println("Motor testing complete!");
}
