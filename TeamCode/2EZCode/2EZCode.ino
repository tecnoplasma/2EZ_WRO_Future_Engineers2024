// #include <utility/imumaths.h>
#include <Servo.h>
#include <SPI.h>
#include <Pixy2.h>
#include <Pixy2CCC.h>


Servo servo;
Pixy2 pixy; 
// Adafruit_BNO055 bno = Adafruit_BNO055(55);

int motor=2;
int leftUltra=4;
int rightUltra=5;
int frontUltra=6;   

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
  int i;

  pixy.ccc.getBlocks();
  if (pixy.ccc.numBlocks)
  {
    Serial.print("Sig  ");
    Serial.print(pixy.ccc.blocks[0].m_signature);
    Serial.print("  x ");
    Serial.print(pixy.ccc.blocks[0].m_x);
    Serial.print("  y ");
    Serial.print(pixy.ccc.blocks[0].m_y);
    Serial.print("  wid ");
    Serial.print(pixy.ccc.blocks[0].m_width);
    Serial.print("  hei ");
    Serial.print(pixy.ccc.blocks[0].m_height);
    Serial.print("  angle ");
    Serial.print(pixy.ccc.blocks[0].m_angle);
    Serial.print("  index ");
    Serial.print(pixy.ccc.blocks[0].m_index);
    Serial.print("  age ");
    Serial.println(pixy.ccc.blocks[0].m_age);
    if (pixy.ccc.numBlocks>1) {
      Serial.print(pixy.ccc.blocks[1].m_signature);
      Serial.print(pixy.ccc.blocks[1].m_x);
      Serial.print(pixy.ccc.blocks[1].m_y);
      Serial.print(pixy.ccc.blocks[1].m_width);
      Serial.print(pixy.ccc.blocks[1].m_height);
      Serial.print(pixy.ccc.blocks[1].m_angle);
      Serial.print(pixy.ccc.blocks[1].m_index);
      Serial.println(pixy.ccc.blocks[1].m_age);
    }
  
    
     
  }
  
  delay(500);
}
