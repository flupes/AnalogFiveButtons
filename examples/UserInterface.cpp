#include <Wire.h>

#include "ST7565.h"

#include "AnalogFiveButtons.h"

#define BACKLIGHT_LED 3
#define WAIT_LOOP 10

#include "Arduino.h"

void setup();
void loop();
ST7565 glcd(9, 8, 7, 6, 5);

AnalogFiveButtons a5b(A2, 5.0);
word ladder[6] = { 4990, 22100, 9310, 4990, 2100, 1039 };

byte state = 0;

int counter;
unsigned long prevtime;

int squareSize = 12;
int squareSpacing = 24;
int startX = 6;
int startY = 42; 
int buttonsX[5];

char elapsedStr[8];
char blank[8] = "       ";

char timeStr[24];

void setup() {                
  
  // initialize and set the contrast to 0x18
  glcd.begin(0x18);

  // show splashscreen
  glcd.display(); 
  delay(500);
  glcd.clear(); 
  
  // turn on backlight
  pinMode(BACKLIGHT_LED, OUTPUT);
  digitalWrite(BACKLIGHT_LED, HIGH);

  // For I2C
  Wire.begin();

  // For console debugging
  Serial.begin(9600);

  // Configure the ladder
  a5b.setLadder(3.3, ladder);
  a5b.removeState(17);
  a5b.removeState(18);
  a5b.setTiming(20, 3);

  for (int i=0; i<5; i++) {
    buttonsX[5-i-1] = startX+i*squareSpacing;
  }

  counter = 0;
  prevtime = millis();
}

void loop()                     
{
  //glcd.clear();

  a5b.update();

  byte bm = 1;
  for ( int i=0; i<5; i++ ) {
    int state = a5b.getState(bm);
    if ( state ) {
      glcd.fillrect(buttonsX[i], startY, squareSize, squareSize, 1);
    }
    else {
      glcd.fillrect(buttonsX[i]+1, startY+1, squareSize-2, squareSize-2, 0);
      glcd.drawrect(buttonsX[i], startY, squareSize, squareSize, 1);
    }
    bm = bm << 1;
  }
  

  unsigned long newtime = millis();
  int elapsed = newtime-prevtime;
  String str(elapsed, DEC);
  str.toCharArray(elapsedStr, 8);
  prevtime = newtime;

  glcd.drawstring(8, 4, blank);
  glcd.drawstring(8, 4, elapsedStr);

  glcd.display();

  counter++;

  delay(WAIT_LOOP);

}

int main(void)
{
  init();
  
  setup();
  
  for (;;)
    loop();
  
  return 0;
}
