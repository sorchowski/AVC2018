/***********************************************************************

"THE BEER-WARE LICENSE"
As long as you retain this notice you can do whatever you want with this stuff. 
If we meet some day, and you think this stuff is worth it, you can buy me a beer.
************************************************************************/
#include <SoftwareSerial.h>

SoftwareSerial LCD(4,7); // RX, TX

//-------------------------------------------------------------------------------------------
void setup()
{
  LCD.begin(9600);// all SerLCDs come at 9600 Baud by default
  delay(3000);
  resetDefault();
  set_16x2();
  delay(3000);
}
//-------------------------------------------------------------------------------------------
void loop()
{
  delay(500);
  clearScreen();
  selectLineOne();
  int infraredReadValue = analogRead(A2);
  String infraredReadingValueStr = String(infraredReadValue);
  LCD.print(infraredReadingValueStr);
}
//-------------------------------------------------------------------------------------------
void clearScreen()
{
  //clears the screen, you will use this a lot!
  LCD.write(0xFE);
  LCD.write(0x01); 
}
//-------------------------------------------------------------------------------------------
void selectLineOne()
{ 
  //puts the cursor at line 0 char 0.
  LCD.write(0xFE); //command flag
  LCD.write(128); //position
}

void turnDisplayOn()
{
  //this turns the dispaly back ON
  LCD.write(0xFE); //command flag
  LCD.write(12); // 0x0C
}

void set_16x2(){//set character LCD as 16x2
  LCD.write(0x7C); //command flag
  LCD.write(0x04);     //16 char wide
  LCD.write(0x7C); //command flag
  LCD.write(0x06);     //2 lines
  /*NOTE: Make sure to power cycle the serial enabled LCD 
  after re-configuring the # char and lines.*/
}

void resetDefault() {
  for(int i=0; i<20; i++){ //repeats 20x so that the LCD will catch the reset
    LCD.write(0x7C);// special command byte => 0d124 or 0x7C
    LCD.write(0x12);//command to reset the LCD back to 9600
    /*      HOW TO UNBRICK YOUR LCD
    1.)remove power from the LCD so that it goes back to the splash screen
    2.)run the code so that the Arduino runs the resetDefault code
    3.)the LCD will indicate that it has been reset "Reset to 9600"
    4.)power cycle LCD to finish reseting the LCD
    
    Note: There might be some settings that have the backlight
     turned down or you need to change the contrast. If this happens
     just let the code cycle through to turn the brightness back on or
     adjust the potentiometer
     
    If this does not work, the last resort is to try using a Pickit 3 Programmer with
    the steps listed in the SparkFun "Tech Support Tips/Troubleshooting/Common Issues"
    [ https://www.sparkfun.com/tutorials/246#comment-563918fb757b7f100d8b4567 ]
    */
  }
}
