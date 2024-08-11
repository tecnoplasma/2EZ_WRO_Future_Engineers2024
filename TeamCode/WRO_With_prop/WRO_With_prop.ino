// Include necessary libraries
#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Pixy2.h>

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
float previouscontrol=90;

// Create objects for devices
Servo myServo;
MPU6050 mpu;
Pixy2 pixy;

// Constants
const float alpha = 0.98; // You may need to tune this value

// Variables
float accumulatedHeading = 0.0;
boolean Left = false;
unsigned long lastUpdateTime = 0;

// PD constants
float gyroKp = 2.0; // Proportional gain
float gyroKd = 1.0; // Derivative gain
float ultraKp = 2.0; // Proportional gain
float ultraKd = 1.0; // Derivative gain

// PD variables
float gyropreviousError = 0;
float ultrapreviousError = 0;

// Desired heading (in degrees)
float desiredHeading = 0.0;
float desiredDistance = 0.0;
int turns = 0;
int turnMultiplier = 0;
int time = 0;
int currentTime = 0;
long previoustime = 0;

// Define states
enum BotState {
  LookingForObstacles,
  dodgingObstacles,
  findingDirection,
  repositioning
};
BotState currentState = findingDirection;

void setup() {
  // Set up Serial communication for debugging
  Serial.begin(115200);
  pixy.init();

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
  currentMillis = millis();
  pixy.ccc.getBlocks;
  switch(currentState) {
    case findingDirection : {
        setMotorPower(150);
        float heading = getCorrectedHeading();
        float error = desiredHeading - heading;
        float derivative = error - previousError;
        float control = 90 + (Kp * error - (Kd * derivative)/(currentMillis-previoustime));
        if (abs(previouscontrol-control)>20){
          control = previouscontrol;
        }
        setServoPosition(control);
        previousError = error;

        if (readFrontUltrasonic() < 15) {
          if ((readLeftUltrasonic() > readRightUltrasonic())) {
            Left = true;
          } else if ((readLeftUltrasonic() < readRightUltrasonic())) {
            Left = false;
          }
          currentState =  LookingForObstacles;
        }
        Serial.print("Heading: ");
        Serial.print(heading);
        Serial.print(" degrees, Control: ");
        Serial.println(control);
        delay(10);
        break;
      }
    case LookingForObstacles :{
        setMotorPower(150);
        if (Left==true){
          float distance = readLeftUltrasonic();
        }
        else {
          float distance = readRightUltrasonic();
        }
        float Ultraerror = desiredDistance - distance;
        float derivative = Ultraerror - previousError;
        float Ultracontrol = 90 + (ultraKp * error - (ultraKd * derivative)/(currentMillis-previoustime));
        setServoPosition(control);
        previousUltraError = Ultraerror;
        if(pixy.ccc.numBlocks != 0){
          currentState = dodgingObstacles;
        }
      }
    case dodgingObstacles : {
      desiredHeading = getCorrectedHeading();
      if (pixy.ccc.blocks[0].m_signature == 1){
        while (pixy.ccc.numBlocks != 0) {
        setMotorPower(150);
        float heading = getCorrectedHeading();
        float error = desiredHeading - heading;
        float derivative = error - previousError;
        float control = 90 + (Kp * error - (Kd * derivative)/(currentMillis-previoustime));
        setServoPosition(control);
        previousError = error;
        desiredHeading = desiredHeading+0.1;
        }
      }
      if (pixy.ccc.blocks[1].m_signature == 2){
        while (pixy.ccc.numBlocks != 0) {  //also could be while it exceeds a certin number of x or y coordinate
        setMotorPower(150);
        float heading = getCorrectedHeading();
        float error = desiredHeading - heading;
        float derivative = error - previousError;
        float control = 90 + (Kp * error - (Kd * derivative)/(currentMillis-previoustime));
        setServoPosition(control);
        previousError = error;
        desiredHeading = desiredHeading-0.1;
        }
      }
      if(pixy.ccc.numBlocks == 0) {
        currentState =  LookingForObstacles;
      }
    }
    
    }
    previoustime=currentmillis;
    previouscontrol = control;
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