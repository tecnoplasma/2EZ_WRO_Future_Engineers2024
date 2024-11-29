//libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Servo.h>
#include <Pixy2.h> 
#include <Adafruit_NeoPixel.h>
#include <Pixy2I2C.h>
#include <Pixy2SPI_SS.h>

#ifdef __AVR__
#include <avr/power.h> 
#endif

#define LED_PIN 9
#define LED_COUNT 10
#define BRIGHTNESS 255

//declarations
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);


const int rt = A3;
const int re = A2;
const int lt = A1;
const int le = A0;
const int ft = 8;
const int fe = 7;
bool uturn = false;

Adafruit_BNO055 bno = Adafruit_BNO055(55);
// Pixy2 pixy;
Pixy2SPI_SS pixy;
Pixy2I2C pixy2;

Servo steeringServo;
long duration;
int distance;
long prevtime = 0;
// Servo and motor settings
const int SERVO_PIN = 2;
const int STRAIGHT_ANGLE = 82;    // Neutral position for the servo
const int MAX_LEFT = 147;         // Maximum left turn angle
const int MAX_RIGHT = 17;          

// Motor pin assignments
const int mf = 4;    // Motor forward pin
const int mb = 5;    // Motor backward pin
const int me = 3;    // Motor enable pin
const int ms = 6;    // Motor standby pin

// PID control variables
int echoD;

int pleaseworkbno=0;

float Kp = 4.0;                   // Proportional gain
float Ki = 0.0;                   // Integral gain
float Kd = 1.0;                   // Derivative gain
float pKp = 6.5;
float rd;
float ld;
float fd;
int avg = 0;
int sample = 3;
int read;
int forntthreshold = 35;
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
int deviate = 14;

int turns=0;
bool dontSense=false;
bool midTurn=false;
int blocknum = 0;
int speedStr = 130; //120 works if no surprise rule, else 100
int speedTur =  160;  //100 works if no surprise rule, else 80

// Target headings (0, 90, 180, 270)
int targetHeading = 0;

unsigned long turnStartTime2 = 0;
unsigned long turnStartTime = 0;
unsigned long turnDuration = 0;
bool nextCorr=false;
char lastOb='g';
bool joepapa = false;
int centerOffset=0;
bool didWeWait=false;
bool didWeCali=false;



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

enum ParkState{
  leftP,
  rightP,
  park
};
ParkState pState;

void setup() {
  //initilizations
  pinMode(rt, OUTPUT);
  pinMode(lt, OUTPUT);
  pinMode(ft, OUTPUT);
  pinMode(re, INPUT);
  pinMode(le, INPUT);
  pinMode(fe, INPUT);
  Serial.begin(115200); 

  // setting up the servo
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(STRAIGHT_ANGLE);  

  // initializing the BNO055 gyro sensor
  if (!bno.begin()) {
    Serial.println("BNO055 not detected. Check wiring!");
    while (1);
  }
  // initializing the pixy camera
  pixy.init();
  pixy2.init();

  // initializing the neo pixel strip
  strip.begin();
  strip.show();
  strip.setBrightness(BRIGHTNESS);
  colorWipe(strip.Color(255,0,0));

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
  // digitalWrite(mf, HIGH);  
  
}
uint8_t sys, gyro, accel, mag = 0;
void loop() {

  while(pleaseworkbno!=3){
    
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    pleaseworkbno=gyro;
  }
  colorWipe(strip.Color(255,255,255));

  while (turns<=12){
    long currenttime = millis(); //for pid and advanced delays
    
    if (!midTurn) //pids off during dodges and turns
      forward();
    if (!dontSense)  //ultrasonics are off during dodging and turning
      fd=distCalc(ft,fe);
    if (turns>11){
      rd=distCalc(rt,re); 
      ld=distCalc(lt,le);
    }
    pixy.ccc.getBlocks(); 
    if(pixy.ccc.numBlocks >1){
      blocknum = biggerblock();
    }
    else {blocknum=0;}
    if(((pixy.ccc.blocks[blocknum].m_height >47) || (pixy.ccc.blocks[blocknum].m_x > 250)) && pixy.ccc.blocks[blocknum].m_height>pixy.ccc.blocks[blocknum].m_width && fd>120 && pixy.ccc.blocks[blocknum].m_signature==1 && pixy.ccc.blocks[blocknum].m_height >35){
      state=dodgeRight;
    }
    else if(((pixy.ccc.blocks[blocknum].m_height >47) || (pixy.ccc.blocks[blocknum].m_x < 70)) && pixy.ccc.blocks[blocknum].m_height>pixy.ccc.blocks[blocknum].m_width && fd>120 && pixy.ccc.blocks[blocknum].m_signature==2 && pixy.ccc.blocks[blocknum].m_height >35){
      state=dodgeLeft;
    }

    if (fd>120 && !dontSense && !midTurn && currenttime - prevtime <3500){
      rd=distCalc(rt,re);
      ld=distCalc(lt,le);
      if (rd<40){
        centerOffset=20;
      }
      else if (ld<40){
        centerOffset=-20;
      }
      else{
        centerOffset=0;
      }
    }
    if (dontSense || midTurn || (centerOffset!=0 && currenttime-prevtime>1000) || distCalc(ft,fe)<100){
      centerOffset=0;
    }

    if (turns==12 && distCalc(rt,re)<100 && distCalc(lt,le)<100 && !didWeWait && !midTurn && !dontSense){ //&& currenttime-prevtime>3000
      if (distCalc(ft,fe)<150){
        turnStartTime=millis();
        while (millis()-turnStartTime<5000){
          digitalWrite(mf,LOW);
          digitalWrite(mb,LOW);
          //wait for 5 seconds after 3 laps done!
        }
        digitalWrite(mf,HIGH);
        digitalWrite(mb,LOW);
        didWeWait=true;
      }
    }

    switch(state){
      case str:

        if (fd<(forntthreshold-15)){
          rd=distCalc(rt,re);
          ld=distCalc(lt,le);
          if(rd >= rightthreshold){
            state=right;
            setState=right;
            pState=rightP;
          }
          else if(ld >= leftthreshold){
            state=left;
            setState=left;
            pState=leftP;
          }
        }
      break;

      case left:
        if(fd <= forntthreshold && !dontSense  && currenttime - prevtime >3000 && !pixy.ccc.numBlocks){
          if (turns==12){
            turnStartTime=millis();
            while (250>millis()-turnStartTime){
              digitalWrite(mf,LOW);
              digitalWrite(mb,HIGH);
            }
            digitalWrite(mb,LOW);
            digitalWrite(mf,HIGH);
            turnleft();
            turnStartTime=millis();
            while (500>millis()-turnStartTime){
              digitalWrite(mf,LOW);
              digitalWrite(mb,HIGH);
            }
            digitalWrite(mb,LOW);
            digitalWrite(mf,HIGH);
          }
          else {
            analogWrite(me, speedTur);  // Enable motor
            turnleftBack();
            analogWrite(me, speedStr);  // Enable motor
            if (turns==9 && lastOb=='r'){
              turnleft();
              turns=turns-2;
              steeringServo.write(STRAIGHT_ANGLE);
              turnStartTime=millis();
              while (1250>millis()-turnStartTime){
                digitalWrite(mf,LOW);
                digitalWrite(mb,HIGH);
              }
              digitalWrite(mb,LOW);
              delay(250);
              digitalWrite(mf,HIGH);
              setState=right;
              pState=rightP;
              lastOb='n';
            }
          }
        }
      break;

      case right:
        if(fd <= forntthreshold && !dontSense && currenttime - prevtime >3000 && !pixy.ccc.numBlocks){
          if (turns==12) {
            turnStartTime=millis();
            while (250>millis()-turnStartTime){
              digitalWrite(mf,LOW);
              digitalWrite(mb,HIGH);
            }
            digitalWrite(mb,LOW);
            digitalWrite(mf,HIGH);
            turnright();
            turnStartTime=millis();
            while (500>millis()-turnStartTime){
              digitalWrite(mf,LOW);
              digitalWrite(mb,HIGH);
            }
            digitalWrite(mb,LOW);
            digitalWrite(mf,HIGH);
          }
          else{
            analogWrite(me, speedTur);  // Enable motor
            turnrightBack();
            analogWrite(me, speedStr);  // Enable motor
            if (turns==9 && lastOb=='r'){
              turnright();
              turns=turns-2;
              steeringServo.write(STRAIGHT_ANGLE);
              turnStartTime=millis();
              while (1250>millis()-turnStartTime){
                digitalWrite(mf,LOW);
                digitalWrite(mb,HIGH);
              }
              digitalWrite(mb,LOW);
              delay(250);
              digitalWrite(mf,HIGH);
              setState=left;
              pState=leftP;
              lastOb='n';
            }
          }
        }
      break;

      case dodgeLeft: 
        turnDuration=0;

        midTurn=true;
        dontSense=true;
        
        while(pixy.ccc.blocks[blocknum].m_height >75){
          pixy.ccc.getBlocks(); 
          digitalWrite(mf,LOW);
          digitalWrite(mb,HIGH);
        }
        digitalWrite(mb,LOW);
        digitalWrite(mf,HIGH);

        analogWrite(me,120);

        turnStartTime=millis();
        while (pixy.ccc.blocks[blocknum].m_x<260 && pixy.ccc.numBlocks){
          pixy.ccc.getBlocks();  
          steeringServo.write(MAX_LEFT-5);
          if (pixy.ccc.blocks[blocknum].m_height >55 && pixy.ccc.blocks[blocknum].m_x<200){ // && pixy.ccc.blocks[blocknum].m_x>100
            steeringServo.write(MAX_RIGHT);
            digitalWrite(mf,LOW);
            digitalWrite(mb,HIGH);
            delay(100);
          }
          else{
            digitalWrite(mb,LOW);
            digitalWrite(mf,HIGH);
          }

        }
        delay(600);
        turnDuration=millis()-turnStartTime;

        analogWrite(me,speedTur);

        turnStartTime=millis();
        while (min(turnDuration*1.25,1250)+150>millis()-turnStartTime){
          steeringServo.write(MAX_RIGHT+9);
        }

        centerOffset=20;
        turnStartTime=millis();
        while (millis()<turnStartTime+600){
          forward();
        }

        midTurn=false;
        dontSense=false;
        state=setState;
        if (turns==7)
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
        while (pixy.ccc.blocks[blocknum].m_height >75){
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
        while (pixy.ccc.blocks[blocknum].m_x>60 && pixy.ccc.numBlocks){
          pixy.ccc.getBlocks();  
          steeringServo.write(MAX_RIGHT+5);
          if (pixy.ccc.blocks[blocknum].m_height >55 && pixy.ccc.blocks[blocknum].m_x>100){  //&& pixy.ccc.blocks[blocknum].m_x<200
            steeringServo.write(MAX_LEFT);
            digitalWrite(mf,LOW);
            digitalWrite(mb,HIGH);
            delay(100);
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
        while (min(turnDuration*1.5,1500)+150>millis()-turnStartTime){
          steeringServo.write(MAX_LEFT-9);
        }

        centerOffset=-20;
        turnStartTime=millis();
        while (millis()<turnStartTime+600){
          forward();
        }

        midTurn=false;
        dontSense=false;
        state=setState;
        if (turns==7)
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
    

    Serial.print("Heading: ");
    Serial.println(currentHeading);

  }


 
  forntthreshold=37;
  while (turns>=13){
    centerOffset=0;
    // colorWipe(strip.Color(0,255,0));
    long currenttime = millis();
    if (!midTurn) //pids off during dodges and turns
      forward();
    if (!dontSense)  //ultrasonics are off during dodging and turning
      fd=distCalc(ft,fe);
    rd=distCalc(rt,re);
    ld=distCalc(lt,le);

    pixy2.ccc.getBlocks();
    if (pixy2.ccc.blocks[0].m_height> 47 && pixy2.ccc.blocks[0].m_height<pixy2.ccc.blocks[0].m_width && pixy2.ccc.numBlocks){ //height was 40 before just in case
      pState=park;
    }

    switch(pState){

      case rightP:
        if(fd <= forntthreshold && !dontSense  && currenttime - prevtime >2000 && !pixy2.ccc.numBlocks){
          analogWrite(me, speedTur);  // Enable motor
          turnright();
          analogWrite(me, speedStr);  // Enable motor
          turnStartTime=millis();
          while (500>millis()-turnStartTime){
            digitalWrite(mf,LOW);
            digitalWrite(mb,HIGH);
          }
          digitalWrite(mb,LOW);
          delay(250);
          digitalWrite(mf,HIGH);
        }
      break;

      case leftP:
        // colorWipe(strip.Color(255,0,0));
        if(fd <= forntthreshold && !dontSense  && currenttime - prevtime >2000 && !pixy2.ccc.numBlocks){
          // colorWipe(strip.Color(255,255,255));
          analogWrite(me, speedTur);  // Enable motor
          turnleft();
          analogWrite(me, speedStr);  // Enable motor
          turnStartTime=millis();
          while (500>millis()-turnStartTime){
            digitalWrite(mf,LOW);
            digitalWrite(mb,HIGH);
          }
          digitalWrite(mb,LOW);
          delay(250);
          digitalWrite(mf,HIGH);
        }
      break;

      case park:
        dontSense=true;
        // colorWipe(strip.Color(255,0,0));
        if (setState == left){

          
          analogWrite(me,speedTur);
          turnStartTime=millis();
          while (turnStartTime>millis()-(320-pixy2.ccc.blocks[0].m_x)*pKp){ 
            steeringServo.write(MAX_LEFT);
          }
          turnDuration=(320-pixy2.ccc.blocks[0].m_x)*pKp;
          analogWrite(me,200);

          turnStartTime=millis();
          while (turnStartTime>millis()-(turnDuration/1.75)){
            steeringServo.write(MAX_RIGHT);
          }
          turnStartTime=millis();
          while (turnStartTime>millis()-(int)(turnDuration/1.83)){
            digitalWrite(mf,LOW);
            digitalWrite(mb,HIGH);
            steeringServo.write(MAX_LEFT);
          }
          digitalWrite(mf,HIGH);
          digitalWrite(mb,LOW);
          turnStartTime=millis();
          while (turnStartTime>millis()-(3000)){
            steeringServo.write(MAX_RIGHT);
          }


        }
        else if (setState == right){


          analogWrite(me,speedTur);
          turnStartTime=millis();
          while (turnStartTime>millis()-pixy2.ccc.blocks[0].m_x*pKp){ 
            steeringServo.write(MAX_RIGHT);
          }
          turnDuration=pixy2.ccc.blocks[0].m_x*pKp;
          analogWrite(me,200);

          turnStartTime=millis();
          while (turnStartTime>millis()-(turnDuration/1.75)){
            steeringServo.write(MAX_LEFT);
          }
          turnStartTime=millis();
          while (turnStartTime>millis()-(int)(turnDuration/1.83)){
            digitalWrite(mf,LOW);
            digitalWrite(mb,HIGH);
            steeringServo.write(MAX_RIGHT);
          }
          digitalWrite(mf,HIGH);
          digitalWrite(mb,LOW);
          turnStartTime=millis();
          while (turnStartTime>millis()-(3000)){
            steeringServo.write(MAX_LEFT);
          }
          

        }

        digitalWrite(ms,LOW);
          digitalWrite(me,LOW);
          digitalWrite(mf,LOW);
          digitalWrite(mb,LOW);
          while (true){
            //GG. THIS WAS 2EZZ. im joking. it was so hard. i havent slept. in days. :( NAH BUT ITS OVER LETS GO.
          }
      break;


    }


    if (abs(targetHeading-currentHeading)<=10 && dontSense){
      prevtime = currenttime;
      dontSense=false;
    }
  }





  // if (turns>=12){
  //   turnStartTime=millis();
  //   while (turnStartTime>millis()-2000){
  //     forward();
  //   }
  //   // delay(350); //if surpriserule, put delay coz speed is lower
  //   digitalWrite(ms,LOW);
  //   digitalWrite(me,LOW);
  // }


  // Serial.print("Heading: ");
  // Serial.println(currentHeading);
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
  midTurn=true;
    updateTargetHeadingright();
    steeringServo.write(MAX_RIGHT);
    turnStartTime=millis();
    while(currentHeading <= (targetHeading-deviate)){ // && turnStartTime>millis()-2000
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
    midTurn=false;
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
    turnStartTime=millis();
    while(currentHeading <= (targetHeading-deviate) && turnStartTime>millis()-2000){
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
    delay(400);
  digitalWrite(mb,LOW);
  delay(300);
  digitalWrite(mf,HIGH);
  midTurn=false;
}

void turnleft()
{
  steeringServo.write(MAX_LEFT);
  delay(500);
  midTurn=true;
  updateTargetHeadingleft();
  dontSense = true;
  while(currentHeading >= (targetHeading+deviate)){
    // colorWipe(strip.Color(0,255,0));
    bno.getEvent(&event);
    currentHeading = event.orientation.x;
    // colorWipe(strip.Color(0,0,255));
    Serial.print("Heading: ");
    Serial.print(currentHeading);
    Serial.print("| servo position: ");
    Serial.print(newServoPosition);
    Serial.print("| targetHeading ");
    Serial.println(targetHeading-deviate);
  }
  steeringServo.write(STRAIGHT_ANGLE);
  // colorWipe(strip.Color(255,0,0));
  turns++;
  midTurn=false;
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
  turnStartTime=millis();
  while(currentHeading >= (targetHeading+deviate) && turnStartTime>millis()-2000){
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
  delay(400);
  digitalWrite(mb,LOW);
  delay(300);
  digitalWrite(mf,HIGH);
  midTurn=false;
}

// void parking(){

//   while(pixy.ccc.blocks[0].m_signature==3){
    
//   }
// }

void forward(){
  bno.getEvent(&event);
  currentHeading = event.orientation.x;
  error = calculateAngleError(targetHeading, currentHeading);
  // if (turns>12) error-=5; // for drift, it might mess stuff up tho
  error += centerOffset; // for staying in center
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
  if(pixy.ccc.blocks[0].m_width*pixy.ccc.blocks[0].m_height > pixy.ccc.blocks[1].m_width*pixy.ccc.blocks[1].m_height && pixy.ccc.blocks[0].m_height>pixy.ccc.blocks[0].m_width){
    return 0;
  }
  return 1;
}
int betterBlock(){

  pixy.ccc.getBlocks();

  if (turns>12){
    for (int i=0;i<pixy.ccc.numBlocks;i++){
      if(pixy.ccc.blocks[i].m_signature==3){
        return i;
      }
    }
    return 0;
  }



  if (pixy.ccc.numBlocks==1){
    
    if(pixy.ccc.blocks[0].m_signature==1){
      state=dodgeRight;
    }
    else if (pixy.ccc.blocks[0].m_signature==2){
      state=dodgeLeft;
    }

  }
  else if (pixy.ccc.numBlocks==2){

    if(pixy.ccc.blocks[0].m_width*pixy.ccc.blocks[0].m_height > pixy.ccc.blocks[1].m_width*pixy.ccc.blocks[1].m_height){
      return 0;
    }
    return 1;

  }
  else if (pixy.ccc.numBlocks==3){
    int max=-1;
    int index;
    for (int i=0; i<pixy.ccc.numBlocks;i++){
      if (max<pixy.ccc.blocks[i].m_width*pixy.ccc.blocks[i].m_height){
        index=i;
      }
      max=max(max,pixy.ccc.blocks[i].m_width*pixy.ccc.blocks[i].m_height);
    }
  }
}