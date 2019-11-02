
//===== Macro Constants =====//

// Define a default BAUD_RATE of 9600 if none were defined by GALP.
#ifndef BAUD_RATE
    #define BAUD_RATE 9600
#endif

// Define a default SRAM_SIZE of 2048 (The amount of SRAM an Arduino Uno Rev3 has) if none were defined by GALP.
#ifndef SRAM_SIZE
    #define SRAM_SIZE 2048
#endif

//=== Bitmasks that can be used with `statusFlags` to determine various states and settings.
// The type of reading that was most recently taken (or is in progress).
#define READING_TYPE B00000011
// Whether there's a currently a sensor reading in progress.
#define READING_IN_PROGRESS B00000100
// The address of the last analog pin that a reading was taken from.
#define LAST_ANALOG_ADDRESS B00111000
// Whether the pins used by digital sensor 1 should be read from during batch readings.
#define DIGITAL_1_ENABLED B01000000
// Whether the pins used by digital sensor 2 should be read from during batch readings.
#define DIGITAL_2_ENABLED B10000000
// Sets that a single batch reading has been finished and the result needs to be handled.
#define RESULT_READY B00000011

//=== Bitmasks that can be used with `sensorFlags` to determine which ports have sensors connected to them, and which
//=== analog pins are used by connected analog sensors.
// Whether there's currently a sensor connected to the specified port.
#define ANALOG_1_CONNECTED B00000001
#define ANALOG_2_CONNECTED B00000010
#define DIGITAL_1_CONNECTED B00000100
#define DIGITAL_2_CONNECTED B00001000
// Whether the specified analog pin should be read from during batch readings.
#define A0_ENABLED B00010000
#define A1_ENABLED B00100000
#define A2_ENABLED B01000000
#define A3_ENABLED B10000000

//=== Bitmasks for reading and writing from the ADMUX register.
// Prefix for setting the ADC reference voltage to the onboard 5v power supply.
#define ADMUX_PREFIX (1<<REFS0)
// Bitmask that can be used to read the current address stored in the ADMUX register.
#define ADMUX_ADDRESS B00001111
// The ADMUX address for the analog pins.
#define A0_ADDRESS 0
#define A1_ADDRESS 1
#define A2_ADDRESS 2
#define A3_ADDRESS 3
#define A4_ADDRESS 4
#define A5_ADDRESS 5

// Bitmask for starting the ADC.
#define ADCSRA_START (1<<ADEN)|(1<<ADSC)|(1<<ADIE)|(1<<ADPS2)

//===== Global Variables =====//
// Buffer for storing data readings in between transmissions to the client.
volatile uint8_t dataBuffer[(int)(SRAM_SIZE * 0.6)];
// The starting index of the unhandled data in the dataBuffer.
uint32_t dataStartPos = 0;
// The ending index of the unhandled data in the dataBuffer.
volatile uint32_t dataStopPos = 0;
// Bit array that holds flags for the status of various operations and components of the Arduino.
uint8_t statusFlags = B00000000;
// Bit array that holds which sensors connected and whether they're enabled for batch readings.
volatile uint8_t sensorFlags = B00000000;
// Array containing the last ID voltage for every sensor port on the Vernier interface.
uint8_t sensorIDs[4];
// Fields for temporarily storing sensor readings before writing them into the data buffer.
uint32_t datastoreA; uint16_t datastoreB;

//===== Functions =====//
// This function gets called once when the program first starts up. 
void setup()
{
    // Start the serial connection at the specified baud rate
    Serial.begin(BAUD_RATE);

    #ifdef DEBUG
        Serial.write(DEBUG_PREFIX);
        Serial.print(F("Established connection:"));
        Serial.println(millis());
        Serial.write(DEBUG_SUFFIX);
    #endif

    #ifdef BACKUP
        #ifdef DEBUG
            Serial.write(DEBUG_PREFIX);
            Serial.print(F("Saving to EEPROM:"));
            Serial.println(millis());
            Serial.write(DEBUG_SUFFIX);
        #endif

        // Save a backup of all the Arduino's critical registers before we start setting them.
        //TODO save the registers to EEPROM!

        #ifdef DEBUG
            Serial.write(DEBUG_PREFIX);
            Serial.print(F("Backup complete:"));
            Serial.println(millis());
            Serial.write(DEBUG_SUFFIX);
        #endif
    #endif

    // Set analog pins A0,A1,A2,A3,A4,A5 to input mode without changing bit 6 or 7.
    DDRC &= B11000000;
    // Set pins 2,3,4,5,6,7,8,9,10,11,12,13 to input mode, without changing pins 0,1,14,15.
    DDRD &= B00000011;
    DDRB &= B11000000;
    // Set pin 13 (onboard LED) to output mode, without changing any other bits.
    DDRB |= B00100000;

    // Enable the pullup resistor on pin 12 (push button) to invert it's states.
    PORTB |= B00010000;

    // Disable digital readings on analog pins A0,A1,A2,A3,A4,A5, without changing bits 6 and 7.
    DIDR0 |= (1 << ADC0D) | (1 << ADC1D) | (1 << ADC2D) | (1 << ADC3D) | (1 << ADC4D) | (1 << ADC5D);
    // Disable analog comparisons on digital pins.
    DIDR1 |= (1 << AIN0D) | (1 << AIN1D);
    // Ensure the Analog to Digital Converter is enabled.
    ADCSRA |= (1 << ADEN);
    // Disable multiplexing with the analog comparator and auto-triggering of the Analog to Digital conveter.
    ADCSRB = (0 << ACME) | (0 << ADTS0) | (0 << ADTS1) | (0 << ADTS2);
    // Disable the analog comparator.
    ACSR |= (1 << ACD);

    // TODO ensure that all the timer registries are set how we want still!

    #ifdef DEBUG
        Serial.write(DEBUG_PREFIX);
        Serial.print(F("Configured pins:"));
        Serial.println(millis());
        Serial.print(F("DDRB:"));
        Serial.println(DDRB, BIN);
        Serial.print(F("DDRC:"));
        Serial.println(DDRC, BIN);
        Serial.print(F("DDRD:"));
        Serial.println(DDRD, BIN);
        Serial.print(F("PORTB:"));
        Serial.println(PORTB, BIN);
        Serial.print(F("PORTC:"));
        Serial.println(PORTC, BIN);
        Serial.print(F("PORTD:"));
        Serial.println(PORTD, BIN);
        Serial.print(F("DIDR0:"));
        Serial.println(DIDR0, BIN);
        Serial.print(F("DIDR1:"));
        Serial.println(DIDR1, BIN);
        Serial.write(DEBUG_SUFFIX);
    #endif
}

void loop()
{

}

// Convenience method for starting a new analog reading on an analog pin.
inline void startAnalogReading(const uint8_t& address)
{
    // Clear the old 'last analog address' and set it to the new address
    statusFlags = (statusFlags & ~LAST_ANALOG_ADDRESS) | (address << 3);
    // Set the address in the analog to digital converter and start the converter.
    ADMUX = ADMUX_PREFIX | address;
    ADCSRA = ADCSRA_START;
}

// Interrupt service routine that gets called when a reading should be taken during a batch reading.
ISR(TIMER1_COMPA_vect)
{
    #ifdef DEBUG
        Serial.write(DEBUG_PREFIX);
        Serial.print(F("Timer interrupt:"));
        Serial.println(millis());
        Serial.write(DEBUG_SUFFIX);
    #endif

    // Clear old readings out of the datastore.
    datastoreA = 0; datastoreB = 0;
    
    // Read from the digital ports if they're enabled during batch readings.
    if(DIGITAL_1_ENABLED & statusFlags)
    {
        // Copies the states of digital pins 2,3,4,5 into bits 40~43 of the datastore.
        datastoreA |= ((uint32_t)(PORTB & B00111100) << 22);
    }
    if(DIGITAL_2_ENABLED & statusFlags)
    {
        // Copies the states of digital pins 6,7,8,9 into bits 44~47 of the datastore.
        datastoreA |= ((uint32_t)(PORTB & B11000000) << 22) | ((uint32_t)(PORTD & B00000011) << 30);
    }

    // Start a reading from the lowest enabled analog pin, or if no analog readings are enabled,
    // write datastore into the data buffer.
    if(A0_ENABLED & sensorFlags)
    {
        // Clear the old 'last analog address' and set it to address 0 (for pin A0).
        statusFlags = (statusFlags & ~LAST_ANALOG_ADDRESS) | (A0_ADDRESS << 3);
        // Set the address in the analog to digital converter to A0 and start the converter.
        ADMUX = ADMUX_PREFIX | A0_ADDRESS;
        ADCSRA = ADCSRA_START;
    } else
    if(A1_ENABLED & sensorFlags)
    {
        // Clear the old 'last analog address' and set it to address 1 (for pin A1).
        statusFlags = (statusFlags & ~LAST_ANALOG_ADDRESS) | (A1_ADDRESS << 3);
        // Set the address in the analog to digital converter to A1 and start the converter.
        ADMUX = ADMUX_PREFIX | A1_ADDRESS;
        ADCSRA = ADCSRA_START;
    } else
    if(A2_ENABLED & sensorFlags)
    {
        // Clear the old 'last analog address'  and set it to address 2 (for pin A2).
        statusFlags = (statusFlags & ~LAST_ANALOG_ADDRESS) | (A2_ADDRESS << 3);
        // Set the address in the analog to digital converter to A2 and start the converter.
        ADMUX = ADMUX_PREFIX | A2_ADDRESS;
        ADCSRA = ADCSRA_START;
    } else
    if(A3_ENABLED & sensorFlags)
    {
        // Clear the old 'last analog address' and set it to address 3 (for pin A3).
        statusFlags = (statusFlags & ~LAST_ANALOG_ADDRESS) | (A3_ADDRESS << 3);
        // Set the address in the analog to digital converter to A3 and start the converter.
        ADMUX = ADMUX_PREFIX | A3_ADDRESS;
        ADCSRA = ADCSRA_START;
    } else {
        // Set that this reading is ready to be handled by the main loop.
        statusFlags |= RESULT_READY;
    }
}

// Interrupt service routine that gets called whenever an analog reading has been finished.
ISR(ANALOG_COMP_vect)
{
    #ifdef DEBUG
        Serial.write(DEBUG_PREFIX);
        Serial.print(F("ADC interrupt:"));
        Serial.println(millis());
        Serial.write(DEBUG_SUFFIX);
    #endif

    static bool readingHandled = false;

    // Switch off the address of the analog pin that the reading was taken from.
    switch(statusFlags & LAST_ANALOG_ADDRESS)
    {
        case 0:
        {
#ifdef DEBUG
            if(readingHandled)// This should be impossible.
            {
                Serial.write(DEBUG_PREFIX);
                Serial.print(F("!READING HANDLED ON A0!:"));
                Serial.println(millis());
                Serial.write(DEBUG_SUFFIX);
            } else{ // The reading was taken from this pin (A0).
#endif

#ifdef DEBUG
            }
#endif
        }
        case 1:
        {
            
            if(readingHandled) // The reading was taken from a previous pin.
            {
                if((A1_ENABLED & sensorFlags))
                {
                    // If this sensor is enabled during batch readings, take it's reading next.
                }
            } else { // The reading was taken from this pin (A1).

            }
        }
        case 2:
        {
            if(readingHandled) // The reading was taken from a previous pin.
            {
                if((A2_ENABLED & sensorFlags))
                {
                    // If this sensor is enabled during batch readings, take it's reading next.
                }
            } else {// If the reading was taken from this pin (A2).
                
            }
        }
        case 3:
        {
            if(readingHandled) // The reading was taken from a previous pin.
            {
                if((A3_ENABLED & sensorFlags))
                {
                    // If this sensor is enabled during batch readings, take it's reading next.
                }
            } else {// If the reading was taken from this pin (A3).
                
            }
        }
        case 4:// If the reading was taken from pin A4 (resistance measurement).
        {

        }
        case 5:// If the reading was taken from pin A5 (identification voltage).
        {

        }

        #ifdef DEBUG
            default:// This should be impossible.
            {
                Serial.write(DEBUG_PREFIX);
                Serial.print(F("!ANALOG DEFAULT REACHED!:"));
                Serial.println(millis());
                Serial.write(DEBUG_SUFFIX);
                break;
            }
        #endif
    }
}




    












// Interrupt service routine that gets called during a batch reading when a reading should be taken.
// Timer1 ensures this is called at the end of every sample period. It starts taking sensor readings,
// and if no other analog readings need to be taken singals that the reading has finished. Otherwise
// it starts the first necessary analog reading.



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
inline void takeBatchReading()
{
    if(DIGITAL_1_ENABLED & sensorFlags)
    {
        // Copy the values of digital pins 2,3,4,5 into the temporary datastore (specifically bits 40~43).
        // The values of the digital pins are stored in the PORT_ registers, with there being a PORTB,
        // PORTC, and PORTD which map to the same pins that the respective DDR_ registers did in "setup()".
        // Here we use PORTB to get digital pins 0~7, then only pick out 2,3,4,5 with a bitmask. Then we shift
        // the bits by 22 places to where they should be in tempDatastoreA and write them into it.
        tempDatastoreA |= ((uint32_t)(PORTB & B00111100) << 22);
    }
    if(DIGITAL_2_ENABLED & sensorFlags)
    {
        // Copy the values of digital pins 6,7,8,9 into the temporary datastore (specifically bits 44~47).
        // We use the same logic as for digital sensor 1, but here we have to check both PORTB (for bits 6,7), and
        // PORTD (for bits 8,9). Then we shift them and write them into tempDatastoreA.
        tempDatastoreA |= ((uint32_t)(PORTB & B11000000) << 22) | ((uint32_t)(PORTD & B00000011) << 30);
    }
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

//===== Macros =====//
// This program makes use of macros; these are programming instructions that are run when the program is first
// compiled, before the actual program ever starts running. This means they take up no space and don't use any
// processing power, since by the time the program starts running, all the macros will of already been run and
// handled.
//
// Additionally macros can be 'defined' (given a value) outside the program. So the main GALP program can use
// whatever macro values it thinks are most appropiate when uploading the program to the Arduino before starting it.
// The following are all the macros that can be set outside this program:
// BACKUP:      Specifies whether the program should store a backup of it's current settings before it starts running
//              hence providing a fallback in the case this program acidentally bricks the Arduino. It stores a full
//              backup of all the Arduino's registers in EEPROM memory, which persists even after power is
//              disconnected. Setting pin 12 high will cause the program to immediately halt it's execution and do a
//              full restore from the backup. Pin 12 can be set high by pushing the button on the Vernier interface.
//              By default this isn't enabled, but it's a good idea to use while GALP is still in developement.




//TODO do we need a different comparator for the +-10v lines?
//TODO see if using F macro on numeric constants saves memory.
// TODO create DEBUG_PREFIX and DEBUG_SUFFIX