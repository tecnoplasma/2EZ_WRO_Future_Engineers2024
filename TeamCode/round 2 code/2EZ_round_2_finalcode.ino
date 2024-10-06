//libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Servo.h>
#include <Pixy2.h> 
#include <Adafruit_NeoPixel.h>

#ifdef _AVR_
#include <avr/power.h> 
#endif

#define LED_PIN 9
#define LED_COUNT 18
#define BRIGHTNESS 255

//declarations
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRBW + NEO_KHZ800);


const int rt = A3;
const int re = A2;
const int lt = A1;
const int le = A0;
const int ft = 8;
const int fe = 7;
bool uturn = false;

Adafruit_BNO055 bno = Adafruit_BNO055(55);
Pixy2 pixy;

Servo steeringServo;
long duration;
int distance;
long prevtime = 0;
// Servo and motor settings
const int SERVO_PIN = 2;
const int STRAIGHT_ANGLE = 82;    
const int MAX_LEFT = 144;         
const int MAX_RIGHT = 20;         

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
int forntthreshold = 30;
int leftthreshold = 100;  //100 if no surprise //70 if there is
int rightthreshold = 100; //100 if no surprise //70 if there is

float previousError = 0;
float integral = 0;
bool heybro = true;

// Variables to store sensor data and calculations
sensors_event_t event;
float currentHeading = 0;
float error = 0;
float derivative = 0;
float steeringAdjustment = 0;
int newServoPosition = 0;
char mode = 's';
int deviate = 25;

int turns=0;
bool dontSense=false;
bool midTurn=false;
int blocknum = 0;
int speedStr = 160; //120 works if no surprise rule, else 100
int speedTur =  160;  //100 works if no surprise rule, else 80

// Target headings (0, 90, 180, 270)
int targetHeading = 0;

unsigned long turnStartTime = 0;
unsigned long turnDuration = 0;
bool nextCorr=false;
char lastOb='g';
bool joepapa = false;


//function to calculate the shortest turn direction
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

//functions to update the target heading periodically
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

//robot's current state, stationary state machine
enum State{
  str,
  left,
  right,
  dodgeLeft,
  dodgeRight
};
//state objects
State state=str;
State setState=str;

void setup() {
  //initilizations
  pinMode(rt, OUTPUT);
  pinMode(lt, OUTPUT);
  pinMode(ft, OUTPUT);
  pinMode(re, INPUT);
  pinMode(le, INPUT);
  pinMode(fe, INPUT);
  Serial.begin(115200); 

  // initializing the BNO055 gyro sensor
  if (!bno.begin()) {
    Serial.println("BNO055 not detected. Check wiring!");
    while (1);
  }
  // initializing the pixy camera
  pixy.init();

  // initializing the neo pixel strip
  strip.begin();
  strip.show();
  strip.setBrightness(BRIGHTNESS);
  colorWipe(strip.Color(255,255,255));

  // setting up the servo
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(STRAIGHT_ANGLE);  

  // setting up motor pins
  pinMode(mf, OUTPUT);
  pinMode(mb, OUTPUT);
  pinMode(me, OUTPUT);
  pinMode(ms, INPUT_PULLUP); //button pin
  while(digitalRead(ms)==1){} 
  delay(100);

  // initialize motor state
  analogWrite(me, speedStr);  
  digitalWrite(mb, LOW);  
  digitalWrite(mf, HIGH);  
  
}

void loop() {
  while (turns<=12){
    long currenttime = millis(); //for pid and advanced delays

    // if (uturn){
    //   while(currenttime > millis()-500){
    //     forward();
    //   }
    //   turnrightBack();
    //   delay(50);
    //   turnright();
    //   delay(50);
    //   steeringServo.write(MAX_LEFT);
    //   digitalWrite(mf,LOW);
    //   digitalWrite(mb,HIGH);
    //   delay(350);
    //   steeringServo.write(STRAIGHT_ANGLE);
    //   delay(800);
    //   digitalWrite(mb,LOW);
    //   digitalWrite(mf,HIGH);
    //   if(setState == left){
    //     setState = right;
    //   }
    //   else if(setState == right){
    //     setState=left;
    //   }
    //   joepapa = true;
    //   uturn = false;
    // }
    
    if (!midTurn) //pids off during dodges and turns
      forward();
    if (!dontSense)  //ultrasonics are off during dodging and turning
      fd=distCalc(ft,fe);

    pixy.ccc.getBlocks(); 
    if(pixy.ccc.numBlocks >1){
      blocknum = biggerblock();
    }
    else {blocknum=0;}
    if((pixy.ccc.blocks[blocknum].m_signature==1 && pixy.ccc.blocks[blocknum].m_age>5 && pixy.ccc.blocks[blocknum].m_height >50) || (pixy.ccc.blocks[blocknum].m_signature==1 && pixy.ccc.blocks[blocknum].m_age>1 && pixy.ccc.blocks[blocknum].m_x > 270)){
      state=dodgeRight;
    }
    else if((pixy.ccc.blocks[blocknum].m_signature==2 && pixy.ccc.blocks[blocknum].m_age>5 && pixy.ccc.blocks[blocknum].m_height >50) || (pixy.ccc.blocks[blocknum].m_signature==2 && pixy.ccc.blocks[blocknum].m_age>1 && pixy.ccc.blocks[blocknum].m_x < 50)){
      state=dodgeLeft;
    }

    
    if (joepapa){
      state = setState;
      joepapa = false;
    }
    switch(state){
      case str:

        if (fd<forntthreshold){
          rd=distCalc(rt,re);
          ld=distCalc(lt,le);
          if(rd >= rightthreshold){
            state=right;
            setState=right;
          }
          else if(ld >= leftthreshold){
            state=left;
            setState=left;
          }
        }
      break;

      case left:
        if(fd <= forntthreshold && !dontSense  && currenttime - prevtime >3000){
          analogWrite(me, speedTur);  // Enable motor
          turnleftBack();
          analogWrite(me, speedStr);  // Enable motor
          if (turns==8 && lastOb=='r'){
            turnleft();
            setState=right;
            lastOb='n';
          }
        }
      break;

      case right:
        if(fd <= forntthreshold && !dontSense && currenttime - prevtime >3000){
          analogWrite(me, speedTur);  // Enable motor
          turnrightBack();
          analogWrite(me, speedStr);  // Enable motor
          if (turns==8 && lastOb=='r'){
            turnright();
            setState=left;
            lastOb='n';
          }
        }
      break;

      case dodgeLeft: 
        turnDuration=0;

        midTurn=true;
        dontSense=true;
        
        while(pixy.ccc.blocks[blocknum].m_height >70){
          pixy.ccc.getBlocks(); 
          digitalWrite(mf,LOW);
          digitalWrite(mb,HIGH);
        }
        digitalWrite(mb,LOW);
        digitalWrite(mf,HIGH);

        analogWrite(me,120);

        turnStartTime=millis();
        while (pixy.ccc.blocks[blocknum].m_x<290 && pixy.ccc.numBlocks){
          pixy.ccc.getBlocks();  
          steeringServo.write(MAX_LEFT);
          if (pixy.ccc.blocks[blocknum].m_height >60 && pixy.ccc.blocks[blocknum].m_x<200 && pixy.ccc.blocks[blocknum].m_x>100){
            steeringServo.write(MAX_RIGHT);
            digitalWrite(mf,LOW);
            digitalWrite(mb,HIGH);
          }
          else{
            digitalWrite(mb,LOW);
            digitalWrite(mf,HIGH);
          }

        }
        delay(300);
        turnDuration=millis()-turnStartTime;

        analogWrite(me,speedTur);

        turnStartTime=millis();
        while (min(turnDuration*1.5,1000)+200>millis()-turnStartTime){
          steeringServo.write(MAX_RIGHT);
        }
        turnStartTime=millis();
        while (millis()<turnStartTime+600){
          forward();
        }

        midTurn=false;
        dontSense=false;
        state=setState;
        lastOb='g';
      break;
      
      case dodgeRight:
        // if(turns == 8 && heybro){
        //   uturn = true;
        //   heybro = false;
        // }
      
        turnDuration=0;

        midTurn=true;
        dontSense=true;
        while (pixy.ccc.blocks[blocknum].m_height >70){
          pixy.ccc.getBlocks(); 
          digitalWrite(mf,LOW);
          digitalWrite(mb,HIGH);
        }
        digitalWrite(mb,LOW);
        digitalWrite(mf,HIGH);

        // while(pixy.ccc.blocks[0].m_height < 30){
        //   pixy.ccc.getBlocks(); 
        // }
        analogWrite(me,120);

        turnStartTime=millis();
        while (pixy.ccc.blocks[blocknum].m_x>30 && pixy.ccc.numBlocks){
          pixy.ccc.getBlocks();  
          steeringServo.write(MAX_RIGHT);
          if (pixy.ccc.blocks[blocknum].m_height >60 && pixy.ccc.blocks[blocknum].m_x<200 && pixy.ccc.blocks[blocknum].m_x>100){
            steeringServo.write(MAX_LEFT);
            digitalWrite(mf,LOW);
            digitalWrite(mb,HIGH);
          }
          else{
            digitalWrite(mb,LOW);
            digitalWrite(mf,HIGH);
          }
        }
        delay(300);
        turnDuration=millis()-turnStartTime;

        analogWrite(me,speedTur);

        turnStartTime=millis();
        while (min(turnDuration*1.5,1000)+200>millis()-turnStartTime){
          steeringServo.write(MAX_LEFT);
        }

        turnStartTime=millis();
        while (millis()<turnStartTime+600){
          forward();
        }

        midTurn=false;
        dontSense=false;
        state=setState;
        lastOb='r';
      break;
    }
    state=setState;
    
      if (abs(targetHeading-currentHeading)<=10 && dontSense){
        prevtime = currenttime;
        dontSense=false;
      }
      // if (turns>=12){
      //   delay(500); 
      //   digitalWrite(mf,LOW);
      //   digitalWrite(me,LOW);
      //   digitalWrite(ms,LOW);
      // }
    



  }


  // prevtime=0;
  forntthreshold=50;
  while (turns>12){
    if (shouldiPark()){
      dontSense=true;
      if (pixy.ccc.blocks[iwannaPark()].m_height>60){
        if (setState==right){

          analogWrite(me,120);

          turnStartTime=millis();
          while (pixy.ccc.blocks[iwannaPark()].m_x>30 && shouldiPark()){
            pixy.ccc.getBlocks();  
            steeringServo.write(MAX_RIGHT);
          }
          delay(300);
          turnDuration=millis()-turnStartTime;

          turnStartTime=millis();
          while (min(turnDuration*1.5,1500)+600>millis()-turnStartTime){
            steeringServo.write(MAX_LEFT);
          }

          digitalWrite(ms,LOW);

        }

        else{

          analogWrite(me,120);

          turnStartTime=millis();
          while (pixy.ccc.blocks[iwannaPark()].m_x<290 && shouldiPark()){
            pixy.ccc.getBlocks();  
            steeringServo.write(MAX_LEFT);
          }
          delay(300);
          turnDuration=millis()-turnStartTime;

          turnStartTime=millis();
          while (min(turnDuration*1.5,1500)+600>millis()-turnStartTime){
            steeringServo.write(MAX_RIGHT);
          }

          digitalWrite(ms,LOW);

        }
      }
    }

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
      if(distCalc(lt,le)>70 || fd<50){
        analogWrite(me, speedTur);  // Enable motor
        turnleft();
      }
      analogWrite(me, speedStr);  // Enable motor
    }
    else if(fd <= forntthreshold && mode == 'r' && !dontSense && currenttime - prevtime >1000){
      if(distCalc(rt,re)>70 || fd<50){
        analogWrite(me, speedTur);  // Enable motor
        turnright();
      }
      analogWrite(me, speedStr);  // Enable motor
    }

    if (abs(targetHeading-currentHeading)<=10 && dontSense){
      prevtime = currenttime;
      dontSense=false;
    }
  }





  // Serial.print("Heading: ");
  // Serial.print(currentHeading);
  // Serial.print("| ld: ");
  // Serial.print(ld);
  // Serial.print("| fd: ");
  // Serial.print(fd);
  // Serial.print("| rd: ");
  // Serial.print(rd);
  // Serial.print("| Target: ");
  // Serial.print(targetHeading);
  // Serial.print("| Mode: ");
  // Serial.print(mode);
  // Serial.print("| Servo Position: ");
  // Serial.println(newServoPosition);
  // Serial.println("yo:");
  // Serial.println(prevtime);
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
    // digitalWrite(me, LOW); 
}
void stopMotor() {
  // digitalWrite(me, LOW);  // Disable the motor (no power)
  digitalWrite(mf, LOW);  // Ensure no forward movement
  digitalWrite(mb, LOW);  // Ensure no backward movement
  // digitalWrite(ms, LOW);  // Set standby to low (optional, depending on your setup)
}
void turnrightBack()
{
  steeringServo.write(MAX_RIGHT);
  delay(500);
  midTurn=true;
  dontSense=true;

  digitalWrite(mf,LOW);
  digitalWrite(mb,HIGH);
    updateTargetHeadingright();
    steeringServo.write(MAX_LEFT);
    delay (750);
    while(currentHeading <= (targetHeading-deviate)){
      bno.getEvent(&event);
      currentHeading = event.orientation.x;
      Serial.print("Heading: ");
      Serial.print(currentHeading);
      Serial.print("| servo position: ");
      Serial.print(newServoPosition);
      Serial.print("| targetHeading ");
      Serial.println(targetHeading-deviate);
    }
    steeringServo.write(STRAIGHT_ANGLE);
      if(!uturn){
        turns++;
        }
    delay(650);
  digitalWrite(mb,LOW);
  digitalWrite(mf,HIGH);
  midTurn=false;
}
void turnrightBackUturn()
{
  steeringServo.write(MAX_RIGHT);
  delay(500);
  midTurn=true;
  dontSense=true;

  digitalWrite(mf,LOW);
  digitalWrite(mb,HIGH);
    updateTargetHeadingright();
    steeringServo.write(MAX_LEFT);
    delay (750);
    while(currentHeading <= (targetHeading-deviate)){
      bno.getEvent(&event);
      currentHeading = event.orientation.x;
      Serial.print("Heading: ");
      Serial.print(currentHeading);
      Serial.print("| servo position: ");
      Serial.print(newServoPosition);
      Serial.print("| targetHeading ");
      Serial.println(targetHeading-deviate);
    }
    steeringServo.write(STRAIGHT_ANGLE);
    if(!uturn){
      turns++;
      }
    delay(1000);
  digitalWrite(mb,LOW);
  digitalWrite(mf,HIGH);
  midTurn=false;
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
}

void turnleftBack()
{
  steeringServo.write(MAX_LEFT);
  delay(500);
  midTurn=true;
  updateTargetHeadingleft();
  dontSense = true;
  digitalWrite(mf,LOW);
  digitalWrite(mb,HIGH);
  steeringServo.write(MAX_RIGHT);
  delay (750);
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
  steeringServo.write(STRAIGHT_ANGLE);
  turns++;
  delay(650);
  digitalWrite(mf,HIGH);
  digitalWrite(mb,LOW);
  midTurn=false;
}

void parking(){

  while(pixy.ccc.blocks[0].m_signature==3){
    
  }
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

void colorWipe(uint32_t color) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();     
  }
}

int biggerblock(){
  if(pixy.ccc.blocks[0].m_width*pixy.ccc.blocks[0].m_height > pixy.ccc.blocks[1].m_width*pixy.ccc.blocks[1].m_height){
    return 0;
  }
  return 1;
}
// int betterBlock(){

//   pixy.ccc.getBlocks();

//   if (turns>12){
//     for (int i=0;i<pixy.ccc.numBlocks;i++){
//       if(pixy.ccc.blocks[i].m_signature==3){
//         return i;
//       }
//     }
//     return 0;
//   }

//   if (pixy.ccc.numBlocks==1){

//     if(pixy.ccc.blocks[0].m_signature==1){
//       state=dodgeRight;
//     }
//     else if (pixy.ccc.blocks[0].m_signature==2){
//       state=dodgeLeft;
//     }

//   }
//   else if (pixy.ccc.numBlocks==2){

//     if(pixy.ccc.blocks[0].m_width*pixy.ccc.blocks[0].m_height > pixy.ccc.blocks[1].m_width*pixy.ccc.blocks[1].m_height){
//       return 0;
//     }
//     return 1;

//   }
//   else if (pixy.ccc.numBlocks==3){

//   }
// }

bool shouldiPark(){
  pixy.ccc.getBlocks();
  for (int i=0;i<pixy.ccc.numBlocks;i++){
    if(pixy.ccc.blocks[i].m_signature==3){
      return true;
    }
  }
  return false;
}

int iwannaPark(){
  pixy.ccc.getBlocks();
  for (int i=0;i<pixy.ccc.numBlocks;i++){
    if(pixy.ccc.blocks[i].m_signature==3){
      return i;
    }
  }
  return 0;
}