// Include necessary libraries
#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>

// Define pins
int DcPort = 3;  // Example pin for DC motor control
int DcDirPort = 2; // Example pin for DC motor direction control
int ServoPort = 9; // Example pin for Servo motor control
int UltraLeftPortTrig = 6; // Example pin for Ultrasonic left trig
int UltraLeftPortEcho = 7; // Example pin for Ultrasonic left echo
int UltraRightPortTrig = 8; // Example pin for Ultrasonic right trig
int UltraRightPortEcho = 9; // Example pin for Ultrasonic right echo
int UltraFrontPortTrig = 10; // Example pin for Ultrasonic front trig
int UltraFrontPortEcho = 11; // Example pin for Ultrasonic front echo
int gyroPort = 0;  // Gyro connected via I2C

// Create objects for devices
Servo myServo;
MPU6050 mpu;

// Constants
const float alpha = 0.98; // You may need to tune this value

// Variables
float accumulatedHeading = 0.0;
boolean isChecked = false;
unsigned long lastUpdateTime = 0;
unsigned long currentMillis = millis();

// PD constants
float Kp = 2.0; // Proportional gain
float Kd = 1.0; // Derivative gain

// PD variables
float previousError = 0;

// Desired heading (in degrees)
float desiredHeading = 0.0;
int turns = 0;
int turnMultiplier = 0;
int time = 0;
int currentTime = 0;

// Define states
enum BotState {
  MovingStraight,
  Turning
};
BotState currentState = MovingStraight;

void setup() {
  // Set up Serial communication for debugging
  Serial.begin(9600);

  // Set up DC motor control pins
  pinMode(DcPort, OUTPUT);
  pinMode(DcDirPort, OUTPUT);

  // Attach the servo motor
  myServo.attach(ServoPort);

  // Set up Ultrasonic sensor pins
  pinMode(UltraLeftPortTrig, OUTPUT);
  pinMode(UltraLeftPortEcho, INPUT);
  pinMode(UltraRightPortTrig, OUTPUT);
  pinMode(UltraRightPortEcho, INPUT);
  pinMode(UltraFrontPortTrig, OUTPUT);
  pinMode(UltraFrontPortEcho, INPUT);

  // Initialize the MPU6050 gyroscope
  Wire.begin();
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
  }
}

void loop() {
  while (turns <= 12) {
    currentMillis = millis();
    switch (currentState) {
      case MovingStraight: {
        setMotorPower(150);
        float heading = getCorrectedHeading();
        float error = desiredHeading - heading;
        float derivative = error - previousError;
        float control = 90 + (Kp * error - Kd * derivative);
        setServoPosition(control);
        previousError = error;

        if ((readFrontUltrasonic() < 20) && currentMillis-currentTime >1000 ) {
          if ((readLeftUltrasonic() > readRightUltrasonic()) && !isChecked) {
            turnMultiplier = -1;
          } else if ((readLeftUltrasonic() < readRightUltrasonic()) && !isChecked) {
            turnMultiplier = 1;
          }
          currentTime = currentMillis;
          currentState = Turning;
        }
        Serial.print("Heading: ");
        Serial.print(heading);
        Serial.print(" degrees, Control: ");
        Serial.println(control);
        delay(10);
        time = time+1;
        break;
      }

      case Turning: {
        desiredHeading += turnMultiplier * 90;
        currentState = MovingStraight;
        turns=turns+1;
        isChecked = false; // Reset the flag after turning
        break;
      }
    }
  }

  // Main code here, currently empty
}

// Function to set the power of the DC motor
void setMotorPower(int speed) {
  digitalWrite(DcDirPort, speed >= 0 ? HIGH : LOW); // Set direction
  analogWrite(DcPort, abs(speed)); // Set speed (0-255)
}

// Function to set the position of the servo motor
void setServoPosition(float control) {
  // Constrain the control value to be within 30 to 150 degrees
  int servoPosition = constrain(control, 30, 150);
  
  // Set the servo position
  myServo.write(servoPosition);
}

// Function to get the heading from the IMU (MPU6050)
float getCorrectedHeading() {
  // Get the current time
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastUpdateTime) / 1000.0; // Time difference in seconds
  lastUpdateTime = currentTime;

  // Read the gyroscope and accelerometer data
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert gyroscope data to degrees per second
  float gyroZ = gz / 131.0; // Sensitivity factor for 250 degrees/second range

  // Convert accelerometer data to angle (in degrees)
  float accelAngle = atan2(ay, ax) * 180 / PI;

  // Update the accumulated heading using the complementary filter
  accumulatedHeading = alpha * (accumulatedHeading + gyroZ * deltaTime) + (1.0 - alpha) * accelAngle;

  return accumulatedHeading;
}

// Function to read distance from the left ultrasonic sensor
long readLeftUltrasonic() {
  return readUltrasonic(UltraLeftPortTrig, UltraLeftPortEcho);
}

// Function to read distance from the right ultrasonic sensor
long readRightUltrasonic() {
  return readUltrasonic(UltraRightPortTrig, UltraRightPortEcho);
}

// Function to read distance from the front ultrasonic sensor
long readFrontUltrasonic() {
  return readUltrasonic(UltraFrontPortTrig, UltraFrontPortEcho);
}

// Function to read distance from an ultrasonic sensor
long readUltrasonic(int trigPin, int echoPin) {
  // Clear the trigger
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Set the trigger high for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echo pin
  long duration = pulseIn(echoPin, HIGH);

  // Calculate the distance
  long distance = duration * 0.034 / 2;

  return distance;
}