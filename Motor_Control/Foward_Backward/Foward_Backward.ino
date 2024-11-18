// Define pin connections
const int ENA = 13;  // Enable pin for motor speed (PWM)
const int IN1 = 12;  // Input 1 for motor direction
const int IN2 = 14;  // Input 2 for motor direction

void setup() {
  // Set up pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Start Serial for debugging
  Serial.begin(115200);
  Serial.println("Motor control ready");
}

void loop() {
  // Example usage: Spin motor clockwise
  Serial.println("Motor spinning clockwise");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 200);  // Set speed (0-255)

  delay(2000);  // Run for 2 seconds

  // Stop the motor
  Serial.println("Stopping motor");
  analogWrite(ENA, 0);
  delay(1000);

  // Spin motor counter-clockwise
  Serial.println("Motor spinning counter-clockwise");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 200);  // Set speed (0-255)

  delay(2000);  // Run for 2 seconds

  // Stop the motor
  Serial.println("Stopping motor");
  analogWrite(ENA, 0);
  delay(1000);
}
