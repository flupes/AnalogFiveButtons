//#include <stdint.h>

#if ARDUINO >= 100
#include <Arduino.h>    // Is it possible to include less???
#else
#include <Wiring.h>	// for types "byte" and "boolean"
			// wiring.h is required anyway in the 
			// implementation for analogRead
#endif

/** 
    AnalogFiveButtons provides the abstraction layer for a five button
    input device.
    
    AnalogFiveButtons use a *single* analog input to detect the state
    of 5 switches. This is done using an resitor ladder network: each
    closed circuit will change the equivalent resistance of the lower
    portion of the resistor divider.

    The class debounces the signal by comparing n successives
    readings. Finally, the class allow to detect when 2 buttons are
    pressed simultaneously.

    The hardware circuit should be constructed as follow:

    refVoltage -+
                |
	        R0
	        |
    analogIn ---+-------+-------+-------+-------+
		|       |       |       |       |
                R1      R2      R3      R4      R5
		|       |       |       |       |
               |: B1   |: B2   |: B3   |: B4   |: B5
		|       |       |       |       |
    GND --------+-------+-------+-------+-------+

    R0 ~= R3
    R5 < R4 < R3 < R2 < R1

    This class could be used with less than five buttons, the unused
    slots will just never be active and state for these buttons will
    always be up.

    This class could be generalized for a arbitrary number of
    buttons. However, for 3 buttons systems, the gain of the unique
    analog becomes only insignificant. For more than 6 buttons, the
    variation in voltage created by the resitor ladder will be very
    small and may lead to wrong readings. Multiplexers or other
    systems will be better suited for input systems requiring more
    than 5 buttons.
  */
class AnalogFiveButtons {
  
 public:

  /**
     Constructor.
      
      @param analogPin		which pin should be used to read the
				analog signal
      @param defaultAnalogRef	the default analog reference used by your
				Arduino board (5V or 3.3V)

      @note AnalogFiveButtons construct a default resistor ladder. If you 
      are using different resistors or a different reference voltage, you 
      need to use setLadder before using the object.
  */
  AnalogFiveButtons(uint8_t analogPin, float defaultAnalogRef=5);
  
  /** Define the resitor ladder.
      
      @param refVoltagereference	voltage at the top of the ladder
      @param R[]			list of resitor values.
      The resistance are expressed in Omhs. The first element of the
      array (R[0]) is the pull up. R[1] to R[5] are the resitance for the
      switches 1 to 5. Note that because of the coding resitance more
      than 65K cannot be used...
  */
  void setLadder(float refVoltage, uint16_t *R);


  /**
     Configure at the scanning rate and debounce count.
     @param     msSampling      Period at which we sample (in ms)
                                A new state is computed only when msSampling
                                has elapsed. This means that even if multiples
                                "update" have been called within msSampling,
                                only one measurement is still 
     @param     debounceCount   Sets how many measuremens with the same value
                                we desire before considering the state stable.
  */
  void setTiming(uint16_t msSampling, uint8_t debounceCount);

  boolean removeState(byte state);

  const static byte BM_1 = 1;	/** mask defining button 1 */
  const static byte BM_2 = 2;	/** mask defining button 2 */
  const static byte BM_3 = 4;	/** mask defining button 3 */
  const static byte BM_4 = 8;	/** mask defining button 4 */
  const static byte BM_5 = 16;	/** mask defining button 5 */
  const static byte ALL_BUTTONS = BM_1 | BM_2 | BM_3 | BM_4 | BM_5;
  const static byte buttons[5];

  /** Returns if the given button is pressed or not.
      @param button	the button code (BM_1 - BM_5)
      @return		1 if the button is pressed, 0 otherwise
   */
  boolean getState(byte button);

  /**
     Returns if the given button has been pressed.
      @param button button, or button combination to get state for
      @return       true if the button transitioned from up to down
   */
  byte buttonPressed(byte button);

  /**
     Mark the given button as a processed event.
      @param button button, or button combination to clear
  */
  void clearButton(byte button);

  /**
     Returns the states of all buttons coded on a a char.
   */
  byte getState();

  byte getPressedState();

  /**
     Update the current button state by reading the analog pin
      and computing the resulting state.
   */
  void update();
  
  int analogPin() { return m_analogPin; }

 protected:
  /**
     Compute the voltage output for each state of the ladder, coded
     on 10 bits.
  */
  void computeLadder();

  /**
     Returns the state associated with the given reading of the input
     voltage.
   */
  byte computeState(int analogReading);

  float m_refVoltage;		/** reference voltage at the top of the ladder */
  float m_defaultAnalogRef;	/** default analog reference for the board */
  uint8_t m_analogPin;		/** pin used to read the analog input */
  uint8_t m_debounceCount;	/** Number of succesives readings need to 
                                be identical before we decide the voltage is stable */

  uint16_t m_msSampling;	/** Required delay between 2 successive
                                readings (in ms) */
  uint16_t m_resistors[6];
  byte m_states[16];
  int16_t m_ladder[16];
  byte m_currentStateIndex;
  byte m_buttonPressed;

};

