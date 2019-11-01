
// Define a default BAUD_RATE of 9600 if none were defined by GALP.
#ifndef BAUD_RATE
    #define BAUD_RATE 9600
#endif

// Define a default SRAM_SIZE of 2048 (The amount of SRAM an Arduino Uno Rev3 has) if none were defined by GALP.
#ifndef SRAM_SIZE
    #define SRAM_SIZE 2048
#endif

//===== Macro constants =====//









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

// Bitmask that can be used with sensorFlags to tell if the analog-1 sensor is currently connected.
#define ANALOG_1_CONNECTED B00000001
// Bitmask that can be used with sensorFlags to tell if the analog-2 sensor is currently connected.
#define ANALOG_2_CONNECTED B00000010
// Bitmask that can be used with sensorFlags to tell if the digital-1 sensor is currently connected.
#define DIGITAL_1_CONNECTED B00000100
// Bitmask that can be used with sensorFlags to tell if the digital-2 sensor is currently connected.
#define DIGITAL_2_CONNECTED B00001000
// Bitmask that can be used with sensorFlags to tell if the analog-1 sensor is enabled for batch readings.
#define ANALOG_1_ENABLED B00010000
// Bitmask that can be used with sensorFlags to tell if the analog-2 sensor is enabled for batch readings.
#define ANALOG_2_ENABLED B00100000
// Bitmask that can be used with sensorFlags to tell if the digital-1 sensor is enabled for batch readings.
#define DIGITAL_1_ENABLED B01000000
// Bitmask that can be used with sensorFlags to tell if the digital-2 sensor is enabled for batch readings.
#define DIGITAL_2_ENABLED B10000000

// Bitmask that can be used with statusFlags to tell if a reading is currently being taken.
#define READING_IN_PROGRESS B00000001

// Bitmask that should always be ORd when setting the ADMUX (Analog to Digital Multiplexer).
// This controls settings on how analog readings are taken and the reference voltage they use.
// The first 4 bits of this set that the readings should be stored 'left-adjusted' and that the
// board's internal 5v power supply should be used as a reference voltage. The last 4 bits set
// which analog pin to measure, for more information check out "takeReading()".
#define ADMUX_PREFIX B01000000
// Bitmask that can be applied to the ADMUX register to get the address of the analog pin that
// the reading was taken from.
#define ADMUX_ADDRESS_MASK B00001111
// Multiplex address for pin A0, setting this in ADMUX will make it measure pin A0.
#define A0_ADDRESS B00000000
// Multiplex address for pin A0, setting this in ADMUX will make it measure pin A0.
#define A1_ADDRESS B00000001
// Multiplex address for pin A0, setting this in ADMUX will make it measure pin A0.
#define A2_ADDRESS B00000010
// Multiplex address for pin A0, setting this in ADMUX will make it measure pin A0.
#define A3_ADDRESS B00000011
// Multiplex address for pin A0, setting this in ADMUX will make it measure pin A0.
#define A4_ADDRESS B00000100
// Multiplex address for pin A0, setting this in ADMUX will make it measure pin A0.
#define A5_ADDRESS B00000101


//===== Global constants =====//
// Here we calculate constants that can be accessed anywhere else in the program.

// The amount of space to allocate to the data buffer. We allocate 60% of the system's SRAM for the data buffer.
const uint32_t DATA_BUFFER_SIZE = SRAM_SIZE * 0.6;



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
// Bit array that stores various status information in one centralized variable. The variable is used as follows:
// bit 0:   Stores whether or not a reading is currently being taken. 1 indicates a reading is in progress.
// bit 1:   TODO
// bit 2:   TODO
// bit 3:   TODO
// bit 4:   TODO
// bit 5:   TODO
// bit 6:   TODO
// bit 7:   TODO
volatile uint8_t statusFlags = B00000000;

// Array that stores the sensors "Identifying Voltage". Every Vernier sensor has a specific internal resistance
// built into it that can be measured with analog pin A4 (this is explain in greater detail in the "setup" function
// below). The readings taken from that pin are stored in this array (in MUX address order, so analog1=0, analog2=1,
// digital1=2, and digital2=3), so that the Arduino can periodically compare the current value against this stored
// value to automatically detect when a sensor has been removed or connected to it.
volatile uint8_t[] sensorIDs = new uint8_t[4];

// Temporary variables used to store sensor reading values during batch readings. These should be treated as a single
// 48 bit variable in the order (tempDatastoreA, tempDatastoreB) and are encoded as follows:
// bits 0~9:   Store the value read from pin A0 (0~5v analog sensor 1).
// bits 10~19: Store the value read from pin A1 (-10~10v analog sensor 1).
// bits 20~29: Store the value read from pin A2 (0~5v analog sensor 2).
// bits 30~39: Store the value read from pin A3 (-10~10v analog sensor 2).
// bits 40~43: Store the values of digital pins 2,3,4,5 in order.
// bits 44~47: Store the values of digital pins 6,7,8,9 in order.
uint32_t tempDatastoreA; uint16_t tempDatastoreB;


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


    //===== Disable unnecessary features for performance =====//
    // DIDR0 stands for "Digital Input Disable Register". Normally the Arduino automatically lets you read the analog
    // pins as if they were digital inputs instead. Here we only ever take true analog readings from the analog pins,
    // so we use this register to disable this feature on pins A5,A4,A3,A2,A1,A0 by writing a 1 into the corresponding
    // bit locations. The 6th and 7th bit are unused and should always be 0.
    DIDR0 |= B00111111;

    // ADCSR stands for "Analog to Digital Convert Status Register" and is used to control the chip that takes
    // analog readings. The ADCSR is used to set up auto-triggering of the ADC, which we never use. All the readings
    // we take are manually triggered. So we set the entire register to 0 to disable this. (This also disables input
    // multiplexing, which is complicated and not worth explaining, but we don't need it.)
    ADCSRB = B00000000;


    //===== Starting the serial connection =====//
    // Start the serial connection at the specified baud rate.
    Serial.begin(BAUD_RATE);
}



// TODO documentation
void loop()
{

}




/** Function that writes the temporary datastores into the data buffer for transmission. **/
inline void writeTempDatastores()
{
    // Check if theres enough space to fit the whole datastore in the buffer without wrapping around.
    if(dataStopPos <= DATA_BUFFER_SIZE)
    {
        // If there is, write the datastore into the buffer and increment the stop position by 6 afterwards.
        // To write the datastore into the data buffer we break it into 6 bytes (the datastore has 48 bits,
        // and 1byte is 8 bits, so 48/8=6). We do this by shifting the bits so the 8 bits we want are at the end
        // of the datastore, then we use a bitwise AND to only keep those 8 bits and drop everything else.
        dataBuffer[dataStopPos] =   (tempDatastoreA >> 24) & B11111111;
        dataBuffer[dataStopPos+1] = (tempDataStoreA >> 16) & B11111111;
        dataBuffer[dataStopPos+2] = (tempDatastoreA >> 8)  & B11111111;
        dataBuffer[dataStopPos+3] = tempDataStoreA         & B11111111;
        dataBuffer[dataStopPos+4] = (tempDataStoreB >> 8)  & B11111111;
        dataBuffer[dataStopPos+5] = tempDataStoreB         & B11111111;
        dataStopPos += 6;
    } else {
        // If there isn't enough space to fit everything in the data buffer without needing to wrap around, we check
        // if we need to wrap around after writting every single byte. Otherwise we write the data the same as above.

        // We make a copy of the data stop position that we increment and check after every write.
        static uint32_t tempStopPos = dataStopPos;

        dataBuffer[tempStopPos] = (tempDatastoreA >> 24) & B11111111;
        if(tempStopPos == (DATA_BUFFER_SIZE-1)) {tempStopPos = 0;} else {tempStopPos++;}
        dataBuffer[tempStopPos] = (tempDataStoreA >> 16) & B11111111;
        if(tempStopPos == (DATA_BUFFER_SIZE-1)) {tempStopPos = 0;} else {tempStopPos++;}
        dataBuffer[tempStopPos] = (tempDatastoreA >> 8)  & B11111111;
        if(tempStopPos == (DATA_BUFFER_SIZE-1)) {tempStopPos = 0;} else {tempStopPos++;}
        dataBuffer[tempStopPos] = tempDataStoreA         & B11111111;
        if(tempStopPos == (DATA_BUFFER_SIZE-1)) {tempStopPos = 0;} else {tempStopPos++;}
        dataBuffer[tempStopPos] = (tempDataStoreB >> 8)  & B11111111;
        if(tempStopPos == (DATA_BUFFER_SIZE-1)) {tempStopPos = 0;} else {tempStopPos++;}
        dataBuffer[tempStopPos] = tempDataStoreB         & B11111111;
        if(tempStopPos == (DATA_BUFFER_SIZE-1)) {tempStopPos = 0;} else {tempStopPos++;}

        // After we've finished writing the data into the buffer, update the actual data stop position.
        dataStopPos = tempStopPos;
    }
}

/** Function that takes a reading from every sensor enabled for batch readings, and writes the result into the next
  * available buffer position. **/
inline void takeBatchReading()
{
    // First we clear any old values left in the datastore variables by setting them both to 0.
    tempDatastoreA = 0;
    tempDatastoreB = 0;
    // Also clear the analog read progress counter.
    analogReadStatus = 0;

    // If digital sensor 1 is enabled for batch readings.
    if(DIGITAL_1_ENABLED & sensorFlags)
    {
        // Copy the values of digital pins 2,3,4,5 into the temporary datastore (specifically bits 40~43).
        // The values of the digital pins are stored in the PORT_ registers, with there being a PORTB,
        // PORTC, and PORTD which map to the same pins that the respective DDR_ registers did in "setup()".
        // Here we use PORTB to get digital pins 0~7, then only pick out 2,3,4,5 with a bitmask. Then we shift
        // the bits by 22 places to where they should be in tempDatastoreA and write them into it.
        tempDatastoreA |= ((uint32_t)(PORTB & B00111100) << 22);
    }

    // If digital sensor 2 is enabled for batch readings.
    if(DIGITAL_2_ENABLED & sensorFlags)
    {
        // Copy the values of digital pins 6,7,8,9 into the temporary datastore (specifically bits 44~47).
        // We use the same logic as for digital sensor 1, but here we have to check both PORTB (for bits 6,7), and
        // PORTD (for bits 8,9). Then we shift them and write them into tempDatastoreA.
        tempDatastoreA |= ((uint32_t)(PORTB & B11000000) << 22) | ((uint32_t)(PORTD & B00000011) << 30);
    }

    // If analog sensor 1 is enabled for batch readings.
    if(ANALOG_1_ENABLED & sensorFlags)
    {
        // ADMUX is the "Analog to Digital Multiplexer", it's used to select a reference voltage, and specify
        // which analog pin to take readings off of. Here we use ADMUX_PREFIX (Defined at the beginning of the program)
        // to select the board's 5v power supply as the reference voltage, and then set it to also read from pin A0.
        ADMUX = ADMUX_PREFIX | A0_ADDRESS;
        // ADCSR stands for "Analog to Digital Convert Status Register" and is used to control the chip that takes
        // analog readings. Here we enable enable the chip, start a reading, and tell it to issue an interrupt when
        // it's finished (that way we can process other things while the chip is working).
        // "ADEN" stands for "Analog to Digital ENable",
        // "ADSC" stands for "Analog to Digital Start Conversion"
        // "ADIE" stands for "Analog to Digital Interrupt Enable"
        ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADIE);

    }
    // If analog sensor 1 wasn't enabled for batch readings, but analog sensor 2 was.
    else if(ANALOG_2_ENABLED & sensorFlags)
    {
        // We first set the multiplexer, same as above, but this time select pin A2.
        ADMUX = ADMUX_PREFIX | A2_ADDRESS;
        // Just like above, we enable the chip and start a reading with interrupts enabled.
        ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADIE);
    }
    // If neither analog sensor was enabled for batch readings.
    else
    {
        // Write the temporary datastores into the data buffer, as there's no more data to collect.
        writeTempDatastores();
    }
}

/** TODO **/
inline void handleAnalogReadFinished()
{
    // The address of the analog pin that the reading was taken from.
    static unit8_t currentPin = ADMUX & ADMUX_ADDRESS_MASK;

    // If the reading was taken from pin A0 (analog sensor 1)
    if(currentPin == 0)
    {
        // Write the values into the tempdatastore.

        // 
    }
    // If the reading was taken from pin A1 (analog sensor 1)
    else if(currentPin == 1)
    {

    }
    // If the reading was taken from pin A2 (analog sensor 2)
    else if(currentPin == 2)
    {

    }
    // If the reading was taken from pin A3 (analog sensor 2)
    else if(currentPin == 3)
    {

    }
    // If the reading was taken from pin A4 (resistance measurement)
    else if(currentPin == 4)
    {

    }
    // If the reading was taken from pin A5 (identification voltage)
    else if(currentPin == 5)
    {

    }
}


/** This is an "Interrupt Service Routine". It's a function that will be run whenever TIMER1 reaches it's
  * compare-match value (COMP1A). It quickly takes a reading from all the sensors that are being used and
  * stores it in the data buffer.
  *
  * Within this program, this function is called during batch readings once every sample period, as the
  * sample period is what TIMER1 is setup to compare-match with. **/
ISR(TIMER1_COMPA_vect)
{
    // Take a batch reading from all the enabled sensors.
    takeBatchReading();
}

/** This is another interrupt service routine, but this one will get called whenever an analog reading gets
  * finished. **/
ISR(ANALOG_COMP_vect)
{
    // Handle the analog reading that was finished.
    handleAnalogReadFinished();
}

















/** Function that safely increments the data start position, preserving the circular buffer behavior by wrapping
  * the position back to 0 if it reaches the end of the buffer. **/
inline void incrementDataBufferStartPos()
{
    // If the data start position is at the end of the buffer, wrap it back to the start (0), otherwise just increment
    // it like normal.
    if(dataStartPos == (DATA_BUFFER_SIZE - 1))
    {
        dataStartPos = 0;
    } else {
        dataStartPos++;
    }
}

/** Function that safely increments the data stop position, preserving the circular buffer behavior by wrapping
  * the position back to 0 if it reaches the end of the buffer. **/
inline void incrementDataBufferStopPos()
{
    // If the data stop position is at the end of the buffer, wrap it back to the start (0), otherwise just increment
    // it like normal.
    if(dataStopPos == (DATA_BUFFER_SIZE - 1))
    {
        dataStopPos = 0;
    } else {
        dataStopPos++;
    }
}