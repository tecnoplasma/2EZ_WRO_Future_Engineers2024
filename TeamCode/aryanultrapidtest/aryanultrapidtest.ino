#include <Servo.h>

// Pin definitions
const int triggerPin = 4;
const int echoPin = 5;

// PID constants
double Kp = 1.5;  // Proportional gain
double Ki = 0.5;  // Integral gain
double Kd = 1.0;  // Derivative gain
long lasttime = 0;

// Variables for PID
double previousError = 0;
double integral = 0;
double setpoint = 30.0;  // Desired distance from the obstacle in cm

void setup() {
  // Initialize the Serial Monitor
  Serial.begin(9600);

  // Set the trigger pin as output and the echo pin as input
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Attach the servo to the control pin
}

void loop() {
  long currenttime = millis();
  // Measure the distance using the ultrasonic sensor
  double distance = measureDistance();

  // Calculate the error
  double error = setpoint - distance;

  // Calculate the PID terms
  double derivative = error - previousError;
  double output = 90 + (Kp * error+ (Kd * derivative)/(currenttime-lasttime));

  // Clamp the output to the servo range (0-180 degrees)
  output = constrain(output,0,180);

  // Move the servo to the new position

  // Debugging output to Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" cm, Servo Position: ");
  Serial.println(output);

  // Save the current error for the next loop iteration
  previousError = error;

  // Delay to allow the servo to move
  delay(50);
  long lasttime=currenttime;
}

double measureDistance() {
  // Clear the trigger pin
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);

  // Send a 10-microsecond pulse to the trigger pin
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  // Read the echo pin, and calculate the distance
  long duration = pulseIn(echoPin, HIGH);
  double distance = duration * 0.034 / 2;

  return distance;
}
