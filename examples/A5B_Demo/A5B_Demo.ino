#include "Arduino.h"

#include "AnalogFiveButtons.h"

#define WAIT_LOOP 200

AnalogFiveButtons a5b(A0, 3.3);
word ladder[6] = { 4990, 22100, 9310, 4990, 2100, 909 };

byte state = 0;

int counter = 0;
unsigned long prevtime;

void setup() {                
  // For console debugging
  Serial.begin(115200);

  // Configure the ladder
  a5b.setLadder(3.3, ladder);
  a5b.removeState(17);
  a5b.removeState(18);
  a5b.setTiming(200, 2);

  counter = 0;
  prevtime = millis();
}

void loop()                     
{
  a5b.update();

  byte bm = 1;
  for ( int i=0; i<5; i++ ) {
    Serial.print(" | ");
    Serial.print(a5b.getState(bm), DEC);
    bm = bm << 1;
  }
  Serial.println(" |");
  
  unsigned long newtime = millis();
  int elapsed = newtime-prevtime;

  counter++;

  delay(WAIT_LOOP);

}
