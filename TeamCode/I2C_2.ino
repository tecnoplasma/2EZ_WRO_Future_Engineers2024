//SCL=A5, SDA=A4 used Arduino uno
//Keep the I2C addresses different for both pixy2 using pixymon. No need to change anything in the code.

// #define I2C
// #define UART
// #define SPI_SS

// #ifdef I2C
#include <Pixy2I2C.h>
Pixy2I2C pixy1;  // First Pixy2 instance
// Pixy2I2C pixy2;  // Second Pixy2 instance

// #else 
// #ifdef UART
// #include <Pixy2UART.h>
// Pixy2UART pixy1;
// Pixy2UART pixy2;


#include <Pixy2SPI_SS.h>
// Pixy2SPI_SS pixy1;
Pixy2SPI_SS pixy2;

// #else
// #include <Pixy2.h>
// Pixy2 pixy1;
// Pixy2 pixy2;

// #endif
// #endif

void setup()
{
  Serial.begin(115200);
  Serial.print("Starting...\n");
  
  // Initialize both Pixy2 cameras with different I2C addresses
  pixy1.init();
  pixy2.init();
  
  // Optionally set custom I2C addresses if necessary
  // pixy2.setI2CAddress(0x55); // Set address for second Pixy (0x55 is an example)
}

void loop()
{ 
  int i; 
  
  // Grab blocks from both cameras
  pixy1.ccc.getBlocks();
  pixy2.ccc.getBlocks();
  
  // If there are detected blocks from the first camera, print them
  if (pixy1.ccc.numBlocks)
  {
    Serial.print("Pixy1 Detected ");
    Serial.println(pixy1.ccc.numBlocks);
    for (i=0; i<pixy1.ccc.numBlocks; i++)
    {
      Serial.print("  block ");
      Serial.print(i);
      Serial.print(": ");
      pixy1.ccc.blocks[i].print();
    }
  }

  // If there are detected blocks from the second camera, print them
  if (pixy2.ccc.numBlocks)
  {
    Serial.print("Pixy2 Detected ");
    Serial.println(pixy2.ccc.numBlocks);
    for (i=0; i<pixy2.ccc.numBlocks; i++)
    {
      Serial.print("  block ");
      Serial.print(i);
      Serial.print(": ");
      pixy2.ccc.blocks[i].print();
    }
  }
}
