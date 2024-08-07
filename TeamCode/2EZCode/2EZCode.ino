// #include <utility/imumaths.h>
#include <Servo.h>
#include <SPI.h>
#include <Pixy2.h>
#include <Pixy2CCC.h>


Servo servo;
Pixy2 pixy; 
// Adafruit_BNO055 bno = Adafruit_BNO055(55);

const int motor=2;
const int leftUltra=4;
const int rightUltra=5;
const int frontUltra=6;   

const int greenSig=1;
const int redSig=2;

const int x=0;
const int y=1;
const int width=2;
const int height=3;
const int age=4;

int greenStuff[5];
int redStuff[5];
bool twoReads;



// int 

void setup() {
  pinMode(motor,OUTPUT);
  pinMode(leftUltra,INPUT);
  pinMode(rightUltra,INPUT);
  pinMode(frontUltra,INPUT);


  Serial.begin(115200);
  Serial.println("babe im getting turned on");

  pixy.init();
}

void loop() {
  read();





  









}

static void read() 
{
  pixy.ccc.getBlocks();
  if (pixy.ccc.numBlocks)
  {
    if ((pixy.ccc.blocks[0].m_signature)==greenSig){
      getBulkValue(greenStuff,0);
    }
    if ((pixy.ccc.blocks[0].m_signature)==redSig){
      getBulkValue(redStuff,0);
    }

    if (pixy.ccc.numBlocks>1) {
      twoReads=true;
      if ((pixy.ccc.blocks[1].m_signature)==greenSig){
        getBulkValue(greenStuff,1);
      }
      if ((pixy.ccc.blocks[1].m_signature)==redSig){
        getBulkValue(redStuff,1);
      }
    }  
    else{
      twoReads=false;
    }
  }
}

static void getBulkValue(int sigArr[], int index) 
{
    sigArr[x]=pixy.ccc.blocks[index].m_x;
    sigArr[y]=pixy.ccc.blocks[index].m_y;
    sigArr[width]=pixy.ccc.blocks[index].m_width;
    sigArr[height]=pixy.ccc.blocks[index].m_height;
    sigArr[age]=pixy.ccc.blocks[index].m_age;
}
