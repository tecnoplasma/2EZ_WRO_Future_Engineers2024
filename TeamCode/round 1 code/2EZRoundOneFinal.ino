#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Servo.h>
const int rt = A3;
const int re = A2;
const int lt = A1;
const int le = A0;
const int ft = 8;
const int fe = 7;

// Create BNO055 object
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Create servo object
Servo steeringServo;
long duration;
int distance;
long prevtime = 0;
// Servo and motor settings
const int SERVO_PIN = 2;          // Pin where the servo is connected
const int STRAIGHT_ANGLE = 82;    // Neutral position for the servo
const int MAX_LEFT = 160;         // Maximum left turn angle
const int MAX_RIGHT = 20;         // Maximum right turn angle

// Motor pin assignments
const int mf = 4;    // Motor forward pin
const int mb = 5;    // Motor backward pin
const int me = 3;    // Motor enable pin
const int ms = 6;    // Motor standby pin

// PID control variables
int echoD;
float Kp = 4.0;                   // Proportional gain
float Ki = 0.0;                   // Integral gain
float Kd = 1.0;                   // Derivative gain
float rd;
float ld;
float fd;
int avg = 0;
int sample = 3;
int read;
int forntthreshold = 85;
int leftthreshold = 100;  //100 if no surprise //70 if there is
int rightthreshold = 100; //100 if no surprise //70 if there is

float previousError = 0;
float integral = 0;

// Variables to store sensor data and calculations
sensors_event_t event;
float currentHeading = 0;
float error = 0;
float derivative = 0;
float steeringAdjustment = 0;
int newServoPosition = 0;
char mode = 's';
int deviate = 20;

int turns=0;
bool dontSense=false;

int speedStr = 255; //120 works if no surprise rule, else 100
int speedTur =  230;  //100 works if no surprise rule, else 80

// Target headings (0, 90, 180, 270)
int targetHeading = 0;
// unsigned long lastHeadingChangeTime = 0;   // Timer for changing headings
// const unsigned long headingChangeInterval = 5000; // Change heading every 5 seconds (adjustable)

// Helper function to calculate the shortest turn direction
float calculateAngleError(float target, float current) {
  float error = target - current;

  // Handle angle wrapping
  if (error > 180) {
    error -= 360;
  } else if (error < -180) {
    error += 360;
  }

  return error;
}

// Function to update the target heading periodically
void updateTargetHeadingright() {
  Serial.println("updating right");
    if (targetHeading == 0) {
      targetHeading = 90;
    } else if (targetHeading == 90) {
      targetHeading = 180;
    } else if (targetHeading == 180) {
      targetHeading = 270;
    } else if (targetHeading == 270){
      targetHeading = 0;
    }
}

void updateTargetHeadingleft() {
  Serial.println("updating left");
    if (targetHeading == 0) {
      targetHeading = 270;
    } else if (targetHeading == 270) {
      targetHeading = 180;
    } else if (targetHeading == 180) {
      targetHeading = 90;
    } else if (targetHeading == 90){
      targetHeading = 0;
    }
}

void setup() {
  pinMode(rt, OUTPUT);
  pinMode(lt, OUTPUT);
  pinMode(ft, OUTPUT);
  pinMode(re, INPUT);
  pinMode(le, INPUT);
  pinMode(fe, INPUT);
  Serial.begin(115200); 

  // Initialize the BNO055 sensor
  if (!bno.begin()) {
    Serial.println("BNO055 not detected. Check wiring!");
    while (1);
  }

  // Set up the servo
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(STRAIGHT_ANGLE);  // Set initial servo position

  // Set up motor pins
  pinMode(mf, OUTPUT);
  pinMode(mb, OUTPUT);
  pinMode(me, OUTPUT);
  pinMode(ms, INPUT_PULLUP);
  while(digitalRead(ms)==1){} 
  delay(100);
  
  analogWrite(me, speedStr);  // Enable motor
  digitalWrite(mb, LOW);  // Enable motor
  digitalWrite(mf, HIGH);  // Enable motor
  
  // Give time to calibrate the sensor
  delay(100);
}

void loop() {
  long currenttime = millis();
  forward();
  fd=distCalc(ft,fe);
  if(fd <= forntthreshold && mode == 's'){
    rd=distCalc(rt,re);
    ld=distCalc(lt,le);
    if(rd >= rightthreshold){
      mode = 'r';
    }
    else if(ld >= leftthreshold){
      mode = 'l';
    }
  }
  else if(fd <= forntthreshold && mode == 'l' && !dontSense  && currenttime - prevtime >1000){
    if(distCalc(lt,le)>70 || fd<30){
      analogWrite(me, speedTur);  // Enable motor
      turnleft();
      }
    analogWrite(me, speedStr);  // Enable motor
  }
  else if(fd <= forntthreshold && mode == 'r' && !dontSense && currenttime - prevtime >1000){
    if(distCalc(rt,re)>70 || fd<40){
      analogWrite(me, speedTur);  // Enable motor
      turnright();
      }
    analogWrite(me, speedStr);  // Enable motor
  }

  if (abs(targetHeading-currentHeading)<=10 && dontSense){
    prevtime = currenttime;
    dontSense=false;

  }

  if (turns>=12){
    while (currenttime>millis()-1000){
      forward();
    }
    // delay(350); //if surpriserule, put delay coz speed is lower
    digitalWrite(mf,LOW);
  }
  
  Serial.print("Heading: ");
  Serial.print(currentHeading);
  Serial.print("| ld: ");
  Serial.print(ld);
  Serial.print("| fd: ");
  Serial.print(fd);
  Serial.print("| rd: ");
  Serial.print(rd);
  Serial.print("| Target: ");
  Serial.print(targetHeading);
  Serial.print("| Mode: ");
  Serial.print(mode);
  Serial.print("| Servo Position: ");
  Serial.println(newServoPosition);
  Serial.println("yo:");
  Serial.println(prevtime);
}

void turnright()
{
  
  dontSense=true;
    updateTargetHeadingright();
    steeringServo.write(MAX_RIGHT);
    while(currentHeading <= (targetHeading-deviate)){
      bno.getEvent(&event);
      currentHeading = event.orientation.x;
      // if (abs(targetHeading-currentHeading)<=10)
      //   dontSense=false;
      Serial.print("Heading: ");
      Serial.print(currentHeading);
      Serial.print("| servo position: ");
      Serial.print(newServoPosition);
      Serial.print("| targetHeading ");
      Serial.println(targetHeading-deviate);
    }
    turns++;
    // digitalWrite(me, LOW); 
}

void turnleft()
{
  dontSense = true;
  updateTargetHeadingleft();
  steeringServo.write(MAX_LEFT);
  while(currentHeading >= (targetHeading+deviate)){
    bno.getEvent(&event);
    currentHeading = event.orientation.x;
    Serial.print("Heading: ");
    Serial.print(currentHeading);
    Serial.print("| servo position: ");
    Serial.print(newServoPosition);
    Serial.print("| targetHeading ");
    Serial.println(targetHeading-deviate);
  }
  turns++;
}

void forward(){
  bno.getEvent(&event);
  currentHeading = event.orientation.x;
  error = calculateAngleError(targetHeading, currentHeading);
  integral += error;
  derivative = error - previousError;
  steeringAdjustment = Kp * error + Ki * integral + Kd * derivative;
  steeringAdjustment = constrain(steeringAdjustment, -65, 65);
  newServoPosition = STRAIGHT_ANGLE - steeringAdjustment;
  newServoPosition = constrain(newServoPosition, MAX_RIGHT, MAX_LEFT);
  steeringServo.write(newServoPosition);
  previousError = error;
}

float distCalc(int trigPin,int echoPin){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  echoD=pulseIn(echoPin, HIGH, 14000);
  echoD=(echoD/2) / 29.1;
  if ((echoD > 150 || echoD <= 0) ){
    echoD = 150;  }
  return(echoD);
}

float distCalcAVG(int trigPin,int echoPin){

  for( read = 0; read < sample; read++){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  echoD=pulseIn(echoPin, HIGH, 14000);
  echoD=(echoD/2) / 29.1;
  avg = avg + echoD;
  }
  echoD = avg /sample;
  return(echoD);
}