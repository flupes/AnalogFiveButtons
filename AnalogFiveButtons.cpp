#include "AnalogFiveButtons.h"

//#define A5B_DEBUG 1

const byte AnalogFiveButtons::buttons[] = { BM_1, BM_2, BM_3, BM_4, BM_5 };

AnalogFiveButtons::AnalogFiveButtons(uint8_t analogPin, float defaultAnalogRef) :
  m_defaultAnalogRef(defaultAnalogRef),  m_analogPin(analogPin),
  m_currentStateIndex(0)
{
  m_refVoltage = 5.0;

  m_resistors[0] = 4700;
  m_resistors[1] = 22100;
  m_resistors[2] = 10000;
  m_resistors[3] = 4700;
  m_resistors[4] = 2100;
  m_resistors[5] = 1200;

  m_msSampling = 50;
  m_debounceCount = 2;
  
  m_states[0] = 0;	// No button pressed
  m_states[1] = 1;	// B1 pressed
  m_states[2] = 2;	// B2 pressed
  m_states[3] = 3;	// B1 + B2 pressed
  m_states[4] = 4;	// B3 pressed
  m_states[5] = 5;	// B1 + B3 pressed
  m_states[6] = 6;	// B2 + B3 pressed
  m_states[7] = 8;	// B4 pressed
  m_states[8] = 9;	// B1 + B4 pressed
  m_states[9] = 10;	// B2 + B4 pressed
  m_states[10] = 12;	// B3 + B4 pressed
  m_states[11] = 16;	// B5 pressed
  m_states[12] = 17;	// B1 + B5 pressed
  m_states[13] = 18;	// B2 + B5 pressed
  m_states[14] = 20;	// B3 + B5 pressed
  m_states[15] = 24;	// B4 + B5 pressed

  computeLadder();
}

void AnalogFiveButtons::setTiming(uint16_t msSampling, uint8_t debounceCount)
{
  m_msSampling = msSampling;
  m_debounceCount = debounceCount;
}

void AnalogFiveButtons::setLadder(float refVoltage, uint16_t *R)
{
  m_refVoltage = refVoltage;
  for (byte i=0; i<6; i++) {
    m_resistors[i] = R[i];
  }
  computeLadder();
}

boolean AnalogFiveButtons::removeState(byte state)
{
  for (byte i=0; i<16; i++) {
    if ( m_states[i] == state ) {
      m_ladder[i] = 2048;
      return true;
    }
  }
  return false;
}

void AnalogFiveButtons::computeLadder()
{
  // we have:
  // Req = 1 / ( B1/R1 + B2/R2 + B3/R3 + B4/R4 + B5/R5)
  // notation: Bn/Rn means if Bn down, then 1/Rn, otherwise 0
  // Vout = refVoltage * Req / ( Req + R0 )

  float Req;
  float Vout;
  m_ladder[0] = (int16_t)( 1024.0f*(float)m_refVoltage/(float)m_defaultAnalogRef );
  for (byte i=1; i<16; i++) {
    Req = 1.0 / (
		 ( BM_1 & m_states[i] ? 1.0/(float)m_resistors[1] : 0.0 ) +
		 ( BM_2 & m_states[i] ? 1.0/(float)m_resistors[2] : 0.0 ) +
		 ( BM_3 & m_states[i] ? 1.0/(float)m_resistors[3] : 0.0 ) +
		 ( BM_4 & m_states[i] ? 1.0/(float)m_resistors[4] : 0.0 ) +
		 ( BM_5 & m_states[i] ? 1.0/(float)m_resistors[5] : 0.0 )
		 );
    Vout = (float)m_refVoltage*Req/(Req+(float)m_resistors[0]);
    m_ladder[i] = (int16_t)( 1024.0f*(float)Vout/(float)m_defaultAnalogRef );
  }
#ifdef A5B_DEBUG
  for (byte i=0; i<16; i++) {
    Serial.print("ladder[");
    Serial.print(i, DEC);
    Serial.print("] = ");
    Serial.println(m_ladder[i]);
  }
#endif

}

byte AnalogFiveButtons::computeState(int analogReading)
{
  int error;
  byte index = 0;
  int minError = 1024;
#ifdef A5B_DEBUG
  Serial.print("==== New reading: ");
  Serial.println(analogReading, DEC);
#endif

  for (byte i=0; i<16; i++) {
    error = abs( m_ladder[i] - analogReading );
#ifdef A5B_DEBUG
    Serial.print(i, DEC);
    Serial.print(" : ");
    Serial.print(m_ladder[i], DEC);
    Serial.print(" -> ");
    Serial.println(error, DEC);
#endif
    if ( error < minError ) {
      minError = error;
      index = i;
    }
  }

#ifdef A5B_DEBUG
  Serial.print("  -> index=");
  Serial.print(index);
  Serial.print(" => ");
  Serial.print(m_states[index], DEC);
  Serial.print(" = ");
  Serial.println(m_states[index], BIN);
#endif

  return index;
}

void AnalogFiveButtons::update()
{
  static byte previousStateIndex = 0;
  static int previousReading = 0;
  static unsigned int previousSampleTime = 0;
  static byte counter = 0;

  // Get current time
  unsigned long currentSampleTime = millis();

  // Start to process only if enough time has elapsed
  if ( currentSampleTime > (previousSampleTime+m_msSampling) ) {

    // Read the current analog input pin
    int currentReading = analogRead(m_analogPin);

    // Check if the reading is not varying much from the previous
    if ( abs(previousReading-currentReading) < 8 ) {

      counter++;	// new stable reading
      if ( counter == m_debounceCount ) {
	// compute the new state
	m_currentStateIndex = computeState(currentReading);
        // Detect the button going down
        // if ( m_currentStateIndex != previousStateIndex ) {
        //   m_buttonPressed = 
        //     ~(m_states[previousStateIndex]) & m_states[m_currentStateIndex];
        //   previousStateIndex = m_currentStateIndex;
        // }
        m_buttonPressed |= m_states[m_currentStateIndex];
	counter = 0;
      }

    }
    previousReading = currentReading;
    previousSampleTime = currentSampleTime;

  }
}

boolean AnalogFiveButtons::getState(byte button)
{
  return ( button & m_states[m_currentStateIndex] );
}

boolean AnalogFiveButtons::buttonPressed(byte button)
{
  return ( button & m_buttonPressed );
}

void AnalogFiveButtons::clearButton(byte button)
{
  m_buttonPressed = m_buttonPressed & ~button;
}

byte AnalogFiveButtons::getState()
{
  return m_states[m_currentStateIndex];
}

byte AnalogFiveButtons::getPressedState()
{
  return m_buttonPressed;
}
