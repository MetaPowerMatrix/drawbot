// Motor control pins
const int motorPwmPin = 9;
const int motorDirPin = 8;
const int servoPin = 10;

// Variables to store commands
float speed = 0.0;
float steeringAngle = 0.0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Initialize motor control pins
  pinMode(motorPwmPin, OUTPUT);
  pinMode(motorDirPin, OUTPUT);
  pinMode(servoPin, OUTPUT);
  
  // Stop motors initially
  analogWrite(motorPwmPin, 0);
  digitalWrite(motorDirPin, LOW);
}

void loop() {
  // Check if data is available
  if (Serial.available() > 0) {
    // Read the incoming command
    String command = Serial.readStringUntil('\n');
    
    // Parse the command
    int speedIndex = command.indexOf('S');
    int angleIndex = command.indexOf('A');
    
    if (speedIndex >= 0 && angleIndex >= 0) {
      // Extract speed and steering angle
      speed = command.substring(speedIndex + 1, angleIndex).toFloat();
      steeringAngle = command.substring(angleIndex + 1).toFloat();
      
      // Control motors based on commands
      controlMotors(speed, steeringAngle);
      
      // Send feedback
      Serial.print("Received: Speed=");
      Serial.print(speed);
      Serial.print(", Angle=");
      Serial.println(steeringAngle);
    }
  }
}

void controlMotors(float speed, float steeringAngle) {
  // Set motor direction
  if (speed >= 0) {
    digitalWrite(motorDirPin, HIGH);  // Forward
  } else {
    digitalWrite(motorDirPin, LOW);   // Reverse
    speed = -speed;  // Make speed positive for PWM
  }
  
  // Set motor speed (0-255)
  int pwmValue = map(speed * 100, 0, 100, 0, 255);
  analogWrite(motorPwmPin, pwmValue);
  
  // Set steering angle (map from radians to servo position)
  // Assuming servo range is 0-180 degrees and steering angle is in radians
  int servoValue = map(steeringAngle * 57.3, -45, 45, 45, 135);  // 57.3 = 180/PI
  analogWrite(servoPin, servoValue);
} 