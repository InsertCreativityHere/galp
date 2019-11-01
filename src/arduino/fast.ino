
//===== Macros =====//
// This program makes use of macros; these are programming instructions that are run when the program is first
// compiled, before the actual program ever starts running. This means they take up no space and don't use any
// processing power, since by the time the program starts running, all the macros will of already been run and
// handled.
//
// Additionally macros can be 'defined' (given a value) outside the program. So the main GALP program can use
// whatever macro values it thinks are most appropiate when uploading the program to the Arduino before starting it.
// The following are all the macros that can be set outside this program:
//
// BAUD_RATE:   Sets the speed of the serial interface the Arduino uses to communicate with the client. Within this
//              context the baud rate is equivalent to bits per second. For a list of supported baud rates, check:
//                      https://www.arduino.cc/en/Serial.Begin
//              This defaults to 9600 (the standard baud rate for most connections).
//
// SRAM_SIZE:   Specifies the amount of SRAM memory the Arduino has in bytes. This defaults to 2048, the amount of
//              SRAM contained on the Arduino Uno Rev3 board.
//
// DEBUG:       Specifies whether to run the program in debug mode. //TODO there's no debug mode yet.
//
// BACKUP:      Specifies whether the program should store a backup of it's current settings before it starts running
//              hence providing a fallback in the case this program acidentally bricks the Arduino. It stores a full
//              backup of all the Arduino's registers in EEPROM memory, which persists even after power is
//              disconnected. Setting pin 12 high will cause the program to immediately halt it's execution and do a
//              full restore from the backup. Pin 12 can be set high by pushing the button on the Vernier interface.
//              By default this isn't enabled, but it's a good idea to use while GALP is still in developement.

// This defines a default baud rate of 9600 for the serial connection is one wasn't already defined.
#ifndef BAUD_RATE
    #define BAUD_RATE 9600
#endif

// This defines a default sram_size value of 2048 bytes if one wasn't already defined.
#ifndef SRAM_SIZE
    #define SRAM_SIZE 2048
#endif


// We define some other macro constants for making this code more readable and easier to write.

// Bitmask that can be used to tell if the analog-1 sensor is currently connected.
#define ANALOG_1_CONNECTED B00000001
// Bitmask that can be used to tell if the analog-2 sensor is currently connected.
#define ANALOG_2_CONNECTED B00000010
// Bitmask that can be used to tell if the digital-1 sensor is currently connected.
#define DIGITAL_1_CONNECTED B00000100
// Bitmask that can be used to tell if the digital-2 sensor is currently connected.
#define DIGITAL_2_CONNECTED B00001000
// Bitmask that can be used to tell if the analog-1 sensor is enabled for batch readings.
#define ANALOG_1_ENABLED B00010000
// Bitmask that can be used to tell if the analog-2 sensor is enabled for batch readings.
#define ANALOG_2_ENABLED B00100000
// Bitmask that can be used to tell if the digital-1 sensor is enabled for batch readings.
#define DIGITAL_1_ENABLED B01000000
// Bitmask that can be used to tell if the digital-2 sensor is enabled for batch readings.
#define DIGITAL_2_ENABLED B10000000



//===== Global constants =====//
// Here we calculate constants that can be accessed anywhere else in the program.

// The amount of space to allocate to the data buffer. We allocate 60% of the system's SRAM for the data buffer.
const uint32_t DATA_BUFFER_SIZE = SRAM_SIZE * 0.6



//===== Global Variables =====//
// These are variables that can be accessed from anywhere else in the program, and are stored in the system's SRAM,
// making them behave like expected of any normal variable.

// The buffer that sensor readings are stored in in between transmissions to the client. This is a "circular buffer",
// meaning that data in it wraps around like a circle, this means that when reading the buffer, there is no end,
// instead it just wraps back around to the beginning. This makes it so that data can be continuously written into,
// and erased from the buffer without ever needed to shift elements around (as long as they're also done in order).
// For more information on circular buffers, check out:
//         https://en.wikipedia.org/wiki/Circular_buffer
// Note: every reading taken from the Vernier interface takes up exactly 6 bytes of space.
volatile uint8_t[] dataBuffer = new uint8_t[DATA_BUFFER_SIZE];
// This is the starting position of valid data in the buffer. Since it's a circular buffer, data doesn't start at
// offset 0 like a normal buffer. Instead of actually removing a byte once it's been handled, this variable just
// increments to indicate that the byte at the last position has been handled and hence 'removed'. Once this variable
// reaches the end of the buffer, it wraps around back to index 0.
volatile uint32_t dataStartPos = 0;
// This is the ending position of valid data in the buffer. Whenever a byte is added to the buffer this value
// increments. Once this variable reaches the end of the buffer, it wraps back around to index 0.
volatile uint32_t dataStopPos = 0;

// Bit array that is used to store whether sensors are connected and/or enabled for batch readings.
// A value of 1 indicates either the sensor is connected, or it's enabled for batch readings, and a value of 0
// respectively indicates the opposite. This variable is meant to be used with the "X_#_CONNECTED" and "X_#_ENABLED"
// macros defined above, so that (sensorFlags & X_#_CONNECTED) will tell you if the specified sensor is connected,
// and likewise for "X_#_ENABLED".
volatile uint8_t sensorFlags = B00000000;

// Array that stores the sensors "Identifying Voltage". Every Vernier sensor has a specific internal resistance
// built into it that can be measured with analog pin A4 (this is explain in greater detail in the "setup" function
// below). The readings taken from that pin are stored in this array (in MUX address order, so analog1=0, analog2=1,
// digital1=2, and digital2=3), so that the Arduino can periodically compare the current value against this stored
// value to automatically detect when a sensor has been removed or connected to it.
volatile uint8_t[] sensorIDs = new uint8_t[4];



//===== Program Functions =====//
/** This method gets called once at the very beginning, and is used to setup and setup and construct everything
  * the Arduino will need to function. **/
void setup()
{
    // First we set what each pin is going to be used for, starting by setting them in 'input' or 'output' mode.
    // Input mode means the pin only passively receives voltages as input (like for taking voltage readings from
    // sensors), and output mode means the pin actively supplies and changes it's own voltage (like for changing
    // states on the Vernier interface).
    //
    // Here we set these properties using the DDRs (Data Direction Registers). These just store 8bit binary values
    // where each bit represents what mode a single pin is going to be in, 0 indicates input mode, and 1 indicates
    // output mode. They're encoded in reverse order, so B10000100 here pin 2 and pin 7 are in output mode, and the
    // rest would be in input mode (remember, the pins start counting at pin 0).
    //
    // There are 3 DDRs, DDRB (controls digital pins 0 through 7), DDRC (controls analog pins A0 through A5, the
    // last 2 bits aren't used, since there's no A6 or A7), and DDRD (controls digital pins 8 through 13, again,
    // the last 2 bits aren't used, since there's no pin 14 and 15). Note that the 4 bits that aren't used because
    // no pin exists to use them is only half true. These bits aren't used for the pins, but ARE used for other
    // things, and hence shouldn't be messed with. Additionally digital pins 0 and 1 are reserved for serial
    // communication by the board, and again, shouldn't be messed with. For more information on all this, check out:
    //        https://www.arduino.cc/en/Reference/PortManipulation

    // We set these registers using bitwise operations, for more information on how these work, just look up
    // "bitwise operations". Here we mostly use AND (&) and OR (|). The following is a pragmatic explanation
    // of what they actually do, at least as far as we use them in this project:
    //
    // &= is bitwise AND, it doesn't affect any bits where there's a 1, but will set any bit to 0 where there's
    //    a 0. For example &= B00000110 Will set every bit to 0, except bit 1 and 2 (which aren't changed at all).
    //
    // |= is a bitwise OR, it doesn't affect any bits where there's a 0, but will set any bit to 1 where there's
    //    a 1 (the opposite of bitwise AND). For example |= B11111001 Will set every bit to 1, except bit 1 and 2
    //    (which aren't changed at all).

    // Finally, pins 0 and 1 are reserved by the Arduino for serial communication, the rest of the pins are used
    // by the Vernier interface as follows:
    //     A0~A3 are used for taking readings from the analog sensors.
    //     A4    is used to measure resistances due to the board's components, this value is used to compensate
    //           and calibrate the voltages read from the sensors.
    //     A5    is used to identify sensors. Every sensor has a slightly different internal resistance; readings
    //           from this pin can use this fact to tell what kind of sensor is connected.
    //     2~9   are used for communicating with and taking readings from digital sensors.
    //     10,11 are used for multiplex (mux) addressing. All 4 sensors share A4 and A5 for transmitting resistance
    //           information. So in order to get resistance readings for a single sensor, these pins are used to
    //           tell the Vernier interface the 'address' of the sensor to transmit. The 'MUX ADDR's are printed on
    //           the Vernier interface with each address being 2 bits. The first bit (most significant bit)
    //           corresponds to pin 11, and the second bit (least significant bit) corresponds to pin 10. So, for
    //           example, having pin10=1 and pin11=0 will give the mux address '01', which corresponds to 'ANALOG 2'.
    //     12    This is mapped to the button on the vernier interface.
    //     13    This is mapped to the LED on the vernier interface.
    
    // With all that out of the way, we first set the pin operation modes.

    //===== Set the analog pin modes =====//
    // Clear every analog pin mode (A0~A5) to 0 (input mode). We only take readings from the analog pins, so none
    // of the pins should be in output mode.
    // This sets pins A0,A1,A2,A3,A4,A5 to 0 (input mode), and doesn't change the 6th and 7th bits.
    DDRC &= B11000000;
  
    //===== Set the digital pin modes =====//
    // First clear every digital pin (except 0 and 1) to 0 (input mode), so they're in a known state.
    
    // Sets pins 2,3,4,5,6,7 to 0 (input mode), and doesn't change the modes of pins 0 and 1.
    DDRD &= B00000011;
    // Sets pins 8,9,10,11,12,13 to 0 (input mode), and doesn't change the 6th and 7th bits.
    DDRB &= B11000000;
    
    // Next, we set the pin modes to 1 for the pins that should be in output mode. The only pin that needs to be in
    // output mode is pin13 (for the LED). Each digital sensor uses it's pins differently, so we set the pin modes
    // for each specific sensor later when we detect what kind of sensors are connected to which ports.

    // Sets pin 13 to 1 (output mode), and doesn't change any of the other pin modes.
    DDRB |= B00100000;

    //===== Enable pull up resistors =====//
    // The arduino also has pullup-resistors, which can be used to invert the signal on a digital pin. We need to
    // enable this for the push button so it isn't always on. If you want to know more about this you can read:
    //        https://www.arduino.cc/en/Tutorial/DigitalPins
    // and read the section "Properties of pins configured as INPUT_PULLUP".
    //
    // But basically there's another set of registers called PORT (there's a PORTB, PORTC, and PORTD which correspond
    // to the same pins as the DDRs did). And we just need to set a single bit on pin 12 to 1 to enable this.
    PORTB |= B00010000;
    

    //==== Starting the serial connection =====//
    // Start the serial connection at the specified baud rate.
    Serial.begin(BAUD_RATE);
}



// TODO documentation
void loop()
{

}



/** This is an "Interrupt Service Routine". It's a function that will be run whenever TIMER1 reaches it's
  * compare-match value (COMP1A). It quickly takes a reading from all the sensors that are being used and
  * stores it in the data buffer.
  *
  * Within this program, this function is called during batch readings once every sample period. **/
ISR(TIMER1_COMPA_vect)
{
    // Temporary variables used to store the data of sensor readings until it's placed into the data buffer.
    // These variables should be treated as one 48 bit long variable (tempDataA, tempDataB). It's encoded as:
    // bits 0~9:   Store the value read from pin A0 (0~5v analog sensor 1).
    // bits 10~19: Store the value read from pin A1 (-10~10v analog sensor 1).
    // bits 20~29: Store the value read from pin A2 (0~5v analog sensor 2).
    // bits 30~39: Store the value read from pin A3 (-10~10v analog sensor 2).
    // bits 40~43: Store the values of digital pins 2,3,4,5 in order.
    // bits 44~47: Store the values of digital pins 6,7,8,9 in order.
    static uint32_t tempDataA = B00000000000000000000000000000000;
    static uint16_t tempDataB = B0000000000000000;
    // If analog sensor 1 is enabled for batch readings.
    if(ANALOG_1_ENABLED & sensorFlags)
    {
        // Take an analog reading from A0 and write it into bits 0~9 of the temporary variables.
        tempDataA |= analogRead(A0);
        // Take an analog reading from A1 and write it into bits 10~19 of the tempeorary variables. This means
        // shifting the variable over by 10 bits and casting it to a 32bit value (so there's enough bits to shift).
        tempDataA |= ((uint32_t)analogRead(A1));
    }
    // If analog sensor 2 is enabled for batch readings.
    if(ANALOG_2_ENABLED & sensorFlags)
    {
        // Take an analog reading from A2 and write it into bits 20~29 of the temporary variables.
        tempDataA |=
        // Take an analog reading from A3 and write it into bits 30~39 of the temporary variables.
        tempDataB
    }
    // If digital sensor 1 is enabled for batch readings.
    if(DIGITAL_1_ENABLED & sensorFlags)
    {

    }
    // If digital sensor 2 is enabled for batch readings.
    if(DIGITAL_1_ENABLED & sensorFlags)
    {

    }
}
