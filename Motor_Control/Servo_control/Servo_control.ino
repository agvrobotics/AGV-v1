#include <ESP32Servo.h>  // Use ESP32-compatible Servo library

// Define pins
const int VRX_PIN = 34;   // Analog pin for joystick horizontal movement
const int SERVO_PIN = 25; // PWM pin for the servo

// Servo object
Servo steeringServo;

// Joystick calibration
int minJoystick = 0;
int maxJoystick = 4095;  // ADC range for ESP32 (0 - 4095)

// Servo movement limits
int minAngle = 45;       // Minimum angle for steering (turn left)
int maxAngle = 135;      // Maximum angle for steering (turn right)
int centerAngle = 90;    // Center angle for straight

void setup() {
  // Attach the servo to the specified pin
  steeringServo.attach(SERVO_PIN);
  
  // Set servo to the center position at startup
  steeringServo.write(centerAngle);

  // Start Serial for debugging
  Serial.begin(115200);
  Serial.println("Joystick Steering Control Ready");
}

void loop() {
  // Read the joystick horizontal position (VRX)
  int joystickValue = analogRead(VRX_PIN);

  // Print the raw joystick reading
  Serial.print("Raw Joystick Value: ");
  Serial.println(joystickValue);

  // Map the joystick value to servo angle
  int servoAngle = map(joystickValue, minJoystick, maxJoystick, minAngle, maxAngle);
  
  // Control the servo position based on joystick input
  steeringServo.write(servoAngle);
  
  // Print the mapped servo angle for debugging
  Serial.print("Mapped Servo Angle: ");
  Serial.println(servoAngle);
  
  delay(20); // Small delay for smoother movement
}
