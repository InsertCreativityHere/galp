
//===== Macros =====//

// Define a default baud rate of 9600 if one wasn't already defined by GALP.
#ifndef BAUD_RATE
    #define BAUD_RATE 9600
#endif

// Define a serial buffer size of 256 if one wasn't already defined by GALP.
#ifndef SERIAL_BUFFER_SIZE
    #define SERIAL_BUFFER_SIZE 256
#endif

//===== Global Constants =====//

// Prefix bits that are always appended to the front a command byte.
// Command bytes should always have the form: `(COMMAND_PREFIX | COMMAND_CODE)`
const uint8_t COMMAND_PREFIX = B01100000;

// Prefix bits that are always append to the front of a response byte.
// Response bytes should always have the form `(RESPONSE_PREFIX | COMMAND_CODE)`.
const uint8_t RESPONSE_PREFIX = B01010000;

// Command codes. These make up the last 4 bits of every command byte.
const uint8_t COMMAND_REPORT_ERROR        = B0000; //0
const uint8_t COMMAND_GET_SAMPLE_RATE     = B0001; //1
const uint8_t COMMAND_SET_SAMPLE_RATE     = B0010; //2
const uint8_t COMMAND_GET_PORT_STATES     = B0011; //3
const uint8_t COMMAND_SET_PORT_STATES     = B0100; //4
const uint8_t COMMAND_GET_PIN_STATES      = B0101; //5
const uint8_t COMMAND_SET_PIN_STATES      = B0110; //6
const uint8_t COMMAND_GET_SENSOR_IDS      = B0111; //7
const uint8_t COMMAND_SET_SENSOR_IDS      = B1000; //8
const uint8_t COMMAND_TAKE_READING        = B1001; //9
const uint8_t COMMAND_START_BATCH_READING = B1010; //10
const uint8_t COMMAND_STOP_BATCH_READING  = B1011; //11
const uint8_t COMMAND_SCAN_SENSORS        = B1100; //12
//const uint8_t COMMAND_13_RESERVED         = B1101; //13
const uint8_t COMMAND_READY               = B1110; //14
const uint8_t COMMAND_DEBUG_LOG           = B1111; //15

// MUX (multiplexer) addresses for the Vernier interface ports.
// These are used to set digital pins 10 and 11 when taking readings from pins A4 and A5.
const uint8_t PORT_ADDRESS_ANALOG_1  = B00; //0
const uint8_t PORT_ADDRESS_ANALOG_2  = B01; //1
const uint8_t PORT_ADDRESS_DIGITAL_1 = B10; //2
const uint8_t PORT_ADDRESS_DIGITAL_2 = B11; //3

// Bitmask for getting the current analog pin address from the `ADMUX` register.
const uint8_t ADMUX_ADDRESS_BITMASK = B00000111;
// ADMUX (analog to digital multiplexer) addresses for the Arduino analog pins.
const uint8_t PIN_ADDRESS_ANALOG_0 = B0000; //0
const uint8_t PIN_ADDRESS_ANALOG_1 = B0001; //1
const uint8_t PIN_ADDRESS_ANALOG_2 = B0010; //2
const uint8_t PIN_ADDRESS_ANALOG_3 = B0011; //3
const uint8_t PIN_ADDRESS_ANALOG_4 = B0100; //4
const uint8_t PIN_ADDRESS_ANALOG_5 = B0101; //5

// Bitmasks for getting the pin-mode of digital pins on the Arduino.
// Can be used with `pinModeFlags` and `..._PIN_STATES`s command's payload (the 1st byte of it).
const uint8_t PIN_MODE_BITMASK_DIGITAL_2 = B00000001;
const uint8_t PIN_MODE_BITMASK_DIGITAL_3 = B00000010;
const uint8_t PIN_MODE_BITMASK_DIGITAL_4 = B00000100;
const uint8_t PIN_MODE_BITMASK_DIGITAL_5 = B00001000;
const uint8_t PIN_MODE_BITMASK_DIGITAL_6 = B00010000;
const uint8_t PIN_MODE_BITMASK_DIGITAL_7 = B00100000;
const uint8_t PIN_MODE_BITMASK_DIGITAL_8 = B01000000;
const uint8_t PIN_MODE_BITMASK_DIGITAL_9 = B10000000;

// Bitmasks for getting if an analog pin is enabled on the Arduino.
// Can be used with `enabledFlags` and `..._PIN_STATES`s command's payload (the 2nd byte of it).
const uint8_t PIN_ENABLED_BITMASK_ANALOG_0 = B00000001;
const uint8_t PIN_ENABLED_BITMASK_ANALOG_1 = B00000010;
const uint8_t PIN_ENABLED_BITMASK_ANALOG_2 = B00000100;
const uint8_t PIN_ENABLED_BITMASK_ANALOG_3 = B00001000;

// Bitmasks for getting if a port is enabled on the Vernier interface.
// Can be used with `enabledFlags` and `..._PORT_STATES`s command's payload (the 1st byte of it).
const uint8_t PORT_ENABLED_BITMASK_ANALOG_1  = B00010000;
const uint8_t PORT_ENABLED_BITMASK_ANALOG_2  = B00100000;
const uint8_t PORT_ENABLED_BITMASK_DIGITAL_1 = B01000000;
const uint8_t PORT_ENABLED_BITMASK_DIGITAL_2 = B10000000;

// Bitmasks for getting various information on the current state and status of the Arduino.
// Can be used with `statusFlags`.
const uint8_t CURRENT_ANALOG_PIN_ADDRESS = B00000111;
const uint8_t CURRENT_READING_TYPE       = B00011000;
const uint8_t ADC_INTERRUPT_BITMASK      = B00100000;
const uint8_t TIMER_INTERRUPT_BITMASK    = B01000000;
const uint8_t READING_COMPLETE_BITMASK   = B10000000;

// Bitvalues representing the current type of reading being taken by the Arduino. These can be used when using the
// `CURRENT_READING_TYPE` bitmask as follows: `(statusFlags & CURRENT_READING_TYPE) == READING_TYPE_NONE`.
const uint8_t READING_TYPE_NONE     = B00000000;
const uint8_t READING_TYPE_SINGLE   = B00010000;
const uint8_t READING_TYPE_BATCH    = B00100000;
const uint8_t READING_TYPE_IDENTIFY = B00110000;

// Error codes. These are sent as payloads alongside `REPORT_ERROR` commands.
const uint8_t ERROR_UNHANDLED_TIMER_ISR       = B00000001; //1
const uint8_t ERROR_UNHANDLED_ADC_ISR         = B00000010; //2
const uint8_t ERROR_UNWRAPPED_STOP_POS        = B00000011; //3
const uint8_t ERROR_READING_ALREADY_COMPLETED = B00000100; //4
const uint8_t ERROR_DEFAULT_READING_TYPE      = B00000101; //5

#ifdef DEBUG_MODE
// Debug codes. These are sent as payloads alongside `DEBUG_LOG` commands.
const uint8_t DEBUG_SETUP_START           = B00000001; //1
const uint8_t DEBUG_SETUP_FINISH          = B00000010; //2
const uint8_t DEBUG_VALUE_DDRB            = B00000011; //3
const uint8_t DEBUG_VALUE_DDRC            = B00000100; //4
const uint8_t DEBUG_VALUE_DDRD            = B00000101; //5
const uint8_t DEBUG_VALUE_PORTB           = B00000110; //6
const uint8_t DEBUG_VALUE_PORTC           = B00000111; //7
const uint8_t DEBUG_VALUE_PORTD           = B00001000; //8
const uint8_t DEBUG_VALUE_DIDR0           = B00001001; //9
const uint8_t DEBUG_VALUE_DIDR1           = B00001010; //10
const uint8_t DEBUG_VALUE_ADCSRA          = B00001011; //11
const uint8_t DEBUG_VALUE_ADCSRB          = B00001100; //12
const uint8_t DEBUG_VALUE_ACSR            = B00001101; //13
const uint8_t DEBUG_MACRO_DUMP            = B00001110; //14
const uint8_t DEBUG_VALUE_BAUD_RATE       = B00001111; //15
const uint8_t DEBUG_VALUE_SERIAL_SIZE     = B00010000; //16
const uint8_t DEBUG_VALUE_AVERAGE_COUNT   = B00010001; //17
const uint8_t DEBUG_VALUE_DEBUG_MODE      = B00010010; //18
const uint8_t DEBUG_VALUE_SAFE_MODE       = B00010011; //19
const uint8_t DEBUG_CONNECTION_READY      = B00010100; //20
const uint8_t DEBUG_ANALOG_START_START    = B00010101; //21
const uint8_t DEBUG_ANALOG_START_FINISH   = B00010110; //22
const uint8_t DEBUG_VALUE_ANALOG_ADDRESS  = B00010111; //23
const uint8_t DEBUG_VALUE_STATUSFLAGS     = B00011000; //24
const uint8_t DEBUG_VALUE_ADMUX           = B00011001; //25
const uint8_t DEBUG_TIMER_CHECK_NO        = B00011010; //26
const uint8_t DEBUG_TIMER_CHECK_START     = B00011011; //27
const uint8_t DEBUG_TIMER_CHECK_FINISH    = B00011100; //28
const uint8_t DEBUG_VALUE_AVERAGE_COUNTER = B00011101; //29
const uint8_t DEBUG_VALUE_ENABLEDFLAGS    = B00011110; //30
const uint8_t DEBUG_VALUE_PINMODEFLAGS    = B00011111; //31
#endif

//===== Global Variables =====//

// Stores the current state of various Arduino operations.
volatile uint8_t statusFlags =  B00000000;
// Stores which ports and analog pins are currently enabled; They're all disabled to start.
uint8_t enabledFlags = B00000000;
// Stores whether each digital pin is in input or output mode; They're all in input mode to start.
uint8_t pinModeFlags = B11111111;
// Stores the ID of what sensor is connected to each port, 0 indicates no sensor is connected.
uint8_t sensorIDs[4] = {0, 0, 0, 0};
//  Stores the current sample period the Arduino is operating at in nanoseconds; This is 0.1s to start.
uint32_t samplePeriod = 100000000;
// Stores the starting position of data in the serial output buffer.
uint16_t serialStartPos = 0;
// Stores the ending position of data in the serial output buffer.
volatile uint16_t serialStopPos = 0;
// Buffer for temporarily storing data before writing it to the client.
volatile uint8_t outputBuffer[SERIAL_BUFFER_SIZE];
// Variable for temporarily storing time readings
#ifdef AVERAGE_COUNT
// If averaging is enabled we allocate a buffer and counter for performing said averaging.
    // Buffer for temporarily storing readings to be averaged together.
    uint8_t averagingBuffer[AVERAGE_COUNT * 10];
    // Counter that tracks how many readings are currently stored in the buffer.
    uint8_t averagingCounter = 0;
#else
    // Variables for temporarily storing
    uint32_t timestore; uint32_t datastoreA; uint16_t datastoreB;
#endif

//===== Functions =====//

// Writes an error message into the serial output buffer with the accompanying `COMMAND_REPORT_ERROR` command code.
inline void reportError(const uint8_t& errorCode)
{
    switch(SERIAL_BUFFER_SIZE - serialStopPos)
    {
  #ifdef SAFE_MODE
        case 0: // This shouldn't happen since serialStopPos should wrap around to 0 after reaching SERIAL_BUFFER_SIZE.
            outputBuffer[0] = (RESPONSE_PREFIX | COMMAND_REPORT_ERROR);
            outputBuffer[1] = errorCode;
            outputBuffer[2] = (RESPONSE_PREFIX | COMMAND_REPORT_ERROR);
            outputBuffer[3] = ERROR_UNWRAPPED_STOP_POS;
            serialStopPos = 4;
            return;
  #endif
        case 1: // If there's only 1 free byte left in the output buffer before wrap-around.
            outputBuffer[serialStopPos] = (RESPONSE_PREFIX | COMMAND_REPORT_ERROR);
            outputBuffer[0] = errorCode;
            serialStopPos = 1;
            return;
        case 2: // If there's only 2 free bytes left in the output buffer before wrap-around.
            outputBuffer[serialStopPos] = (RESPONSE_PREFIX | COMMAND_REPORT_ERROR);
            outputBuffer[serialStopPos+1] = errorCode;
            serialStopPos = 0;
            return;
        default: // If there's more than 2 free bytes left in the output buffer before wrap-around.
            outputBuffer[serialStopPos] = (RESPONSE_PREFIX | COMMAND_REPORT_ERROR);
            outputBuffer[serialStopPos+1] = errorCode;
            serialStopPos += 2;
            return;
    }
}

// Starts an analog reading on the specified analog pin.
inline void startAnalogReading(const uint8_t& address)
{
  #ifdef DEBUG_MODE
  {
    // Log a copy of all the registry values and variables before starting the next analog reading.
    const byte values[] = {RESPONSE_PREFIX | COMMAND_DEBUG_LOG, DEBUG_ANALOG_START_START, 0, 0, 0, 0,
                           DEBUG_VALUE_ANALOG_ADDRESS, address, DEBUG_VALUE_STATUSFLAGS, statusFlags,
                           DEBUG_VALUE_ADMUX, ADMUX, DEBUG_VALUE_ADCSRA, ADCSRA};
    const uint32_t time = micros();
    values[2] = time         & B11111111;
    values[3] = (time >> 8)  & B11111111;
    values[4] = (time >> 16) & B11111111;
    values[5] = (time >> 24) & B11111111;
    Serial.write(values, 14);
  }
  #endif

    // Clear the old analog address and write the new one into `statusFlags`.
    statusFlags = (statusFlags & ~CURRENT_ANALOG_PIN_ADDRESS) | address;
    // Set the address in the ADC and set for it to use the onboard 5v power rail as the reference voltage.
    ADMUX = (1 << REFS0) | address;
    // Enables the ADC, starts a conversion with interrupts enabled, and sets the ADC clock to a multiplier of 16.
    ADCSRA = (1 << ADEN)|(1 << ADSC)|(1 << ADIE)|(1 << ADPS2);

  #ifdef DEBUG_MODE
  {
    // Log a copy of all the registry values and variables after the next analog reading was started.
    const byte values[] = {RESPONSE_PREFIX | COMMAND_DEBUG_LOG, DEBUG_ANALOG_START_FINISH, 0, 0, 0, 0,
                           DEBUG_VALUE_ANALOG_ADDRESS, address, DEBUG_VALUE_STATUSFLAGS, statusFlags,
                           DEBUG_VALUE_ADMUX, ADMUX, DEBUG_VALUE_ADCSRA, ADCSRA};
    const uint32_t time = micros();
    values[2] = time         & B11111111;
    values[3] = (time >> 8)  & B11111111;
    values[4] = (time >> 16) & B11111111;
    values[5] = (time >> 24) & B11111111;
    Serial.write(values, 14);
  }
  #endif
}

// Checks if there's been an ADC interrupt and handles it if there has been. Returns true if an interrupt was handled.
inline bool handleAnalogInterrupt()// TODO NEEDS DEBUG_MODE/SAFE_MODE
{
    // Do nothing if there isn't an ADC interrupt to handle.
    if(!(statusFlags & ADC_INTERRUPT_BITMASK))
    {
        return false;
    }

    switch()

  #ifdef AVERAGE_COUNT
//TODO
  #else
//TODO
  #endif

    // Clear the ADC interrupt flag.
    statusFlags &= ~ADC_INTERRUPT_BITMASK;
    return true;
}

// Checks if there's been a timer interrupt and handles it if there has been. Returns true if an interrupt was handled.
// Timer interrupts are only used for batch readings.
inline bool handleTimerInterrupt()
{
    // Do nothing if there isn't a timer interrupt to handle.
    if(!(statusFlags & TIMER_INTERRUPT_BITMASK))
    {
      #ifdef DEBUG_MODE
      {
        // Log that there weren't any timer interrupts to handle.
        const byte values[] = {RESPONSE_PREFIX | COMMAND_DEBUG_LOG, DEBUG_TIMER_CHECK_NO, 0, 0, 0, 0};
        const uint32_t time = micros();
        values[2] = time         & B11111111;
        values[3] = (time >> 8)  & B11111111;
        values[4] = (time >> 16) & B11111111;
        values[5] = (time >> 24) & B11111111;
        Serial.write(values, 14);
      }
      #endif
        return false;
    }

  #ifdef DEBUG_MODE
  {
  #ifdef AVERAGE_COUNT
    const uint8_t AVERAGE_COUNT_HOLDER = AVERAGE_COUNT;
  #else
    const uint8_t AVERAGE_COUNT_HOLDER = 0;
  #endif
    // Log a copy of the registers and variables before the timer interrupt is handled.
    const byte values[] = {RESPONSE_PREFIX | COMMAND_DEBUG_LOG, DEBUG_TIMER_CHECK_START, 0, 0, 0, 0,
                           DEBUG_VALUE_AVERAGE_COUNTER, averagingCounter, DEBUG_VALUE_AVERAGE_COUNT, AVERAGE_COUNT_HOLDER,
                           DEBUG_VALUE_PINMODEFLAGS, pinModeFlags, DEBUG_VALUE_ENABLEDFLAGS, enabledFlags,
                           DEBUG_VALUE_STATUSFLAGS, statusFlags};
    const uint32_t time = micros();
    values[2] = time         & B11111111;
    values[3] = (time >> 8)  & B11111111;
    values[4] = (time >> 16) & B11111111;
    values[5] = (time >> 24) & B11111111;
    Serial.write(values, 16);
  }
  #endif

  #ifdef AVERAGE_COUNT
    // Store the time that the reading was started at.
    uint32_t time = micros();
    averagingBuffer[averagingCounter]   = time         & B11111111;
    averagingBuffer[averagingCounter+1] = (time >> 8)  & B11111111;
    averagingBuffer[averagingCounter+2] = (time >> 16) & B11111111;
    averagingBuffer[averagingCounter+3] = (time >> 24) & B11111111;

    if(PORT_ENABLED_BITMASK_DIGITAL_1 & enabledFlags)
    {
        // Copies the values of digital pins 2,3,4,5 (only those in input mode) into bits 0~3 of the averaging entry.
        averagingBuffer[averagingCounter+4] = ((PORTB & B00111100) >> 2) & pinModeFlags;
    }

    if(PORT_ENABLED_BITMASK_DIGITAL_2 & enabledFlags)
    {
        // Copies the values of digital pins 6,7,8,9 (only those in input mode) into bits 4~7 of the averaging entry.
        averagingBuffer[averagingCounter+4] |= ((PORTB & B11000000) >> 2) | ((PORTD & B00000011) << 6) & pinModeFlags;
    }
  #else
    // Clear the old temporary datastores.
    datastoreB = 0; datastoreA = 0;

    // Store the time that the reading was started at.
    timestore = micros();

    if(PORT_ENABLED_BITMASK_DIGITAL_1 & enabledFlags)
    {
        // Copies the values of digital pins 2,3,4,5 (only those in input mode) into bits 0~3 of the datastore.
        datastoreA = ((PORTB & B00111100) >> 2) & pinModeFlags;
    }

    if(PORT_ENABLED_BITMASK_DIGITAL_2 & enabledFlags)
    {
        // Copies the values of digital pins 6,7,8,9 (only those in input mode) into bits 4~7 of the datastore.
        datastoreA |= ((PORTB & B11000000) >> 2) | ((PORTD & B00000011) << 6) & pinModeFlags;
    }
  #endif

    // Start a reading on the lowest address analog pin that's enabled, or if none are enabled, end the reading.
    if(PIN_ENABLED_BITMASK_ANALOG_0 & enabledFlags)
    {
        startAnalogReading(PIN_ADDRESS_ANALOG_0);
    } else
    if(PIN_ENABLED_BITMASK_ANALOG_1 & enabledFlags)
    {
        startAnalogReading(PIN_ADDRESS_ANALOG_1);
    } else
    if(PIN_ENABLED_BITMASK_ANALOG_2 & enabledFlags)
    {
        startAnalogReading(PIN_ADDRESS_ANALOG_2);
    } else
    if(PIN_ENABLED_BITMASK_ANALOG_3 & enabledFlags)
    {
        startAnalogReading(PIN_ADDRESS_ANALOG_3);
    } else {
      #ifdef SAFE_MODE
        // Report an error if the reading was already marked as completed.
        if(statusFlags & READING_COMPLETE_BITMASK)
        {
            reportError(ERROR_READING_ALREADY_COMPLETED);
        }
      #endif

        // Set that this reading is finished.
        statusFlags |= READING_COMPLETE_BITMASK;
    }

    // Clear the timer interrupt flag
    statusFlags &= ~TIMER_INTERRUPT_BITMASK;

  #ifdef DEBUG_MODE
  {
  #ifdef AVERAGE_COUNT
    const uint8_t AVERAGE_COUNT_HOLDER = AVERAGE_COUNT;
  #else
    const uint8_t AVERAGE_COUNT_HOLDER = 0;
  #endif
    // Log a copy of the registers and variables after the timer interrupt has been handled.
    const byte values[] = {RESPONSE_PREFIX | COMMAND_DEBUG_LOG, DEBUG_TIMER_CHECK_FINISH, 0, 0, 0, 0,
                           DEBUG_VALUE_AVERAGE_COUNTER, averagingCounter, DEBUG_VALUE_AVERAGE_COUNT, AVERAGE_COUNT_HOLDER,
                           DEBUG_VALUE_PINMODEFLAGS, pinModeFlags, DEBUG_VALUE_ENABLEDFLAGS, enabledFlags,
                           DEBUG_VALUE_STATUSFLAGS, statusFlags};
    const uint32_t time = micros();
    values[2] = time         & B11111111;
    values[3] = (time >> 8)  & B11111111;
    values[4] = (time >> 16) & B11111111;
    values[5] = (time >> 24) & B11111111;
    Serial.write(values, 16);
  }
  #endif

    return true;
}

// Checks if a reading has been completed and handles it if there has been. Returns true ifa reading was handled.
inline bool handleReadingCompleted()// TODO NEEDS DEBUG_MODE/SAFE_MODE
{
    // Do nothing if there isn't a completed reading to handle.
    if(!(statusFlags & READING_COMPLETE_BITMASK))
    {
        return false;
    }

  #ifdef AVERAGE_COUNT
    // If enough readings have been taken to average together, take their average. 
    if(averagingCounter == (AVERAGE_COUNT - 1))
    {
        // Average all the readings. //TODO
        
        // Write the results into the serial output buffer.
    } else {
        averagingCounter++;
    }
  #else
    // Copy the data out of the temporary datastores and into the serial output buffer.
    switch()
  #endif

    // Clear the reading complete flag.
    statusFlags &= ~READING_COMPLETE_BITMASK;
    return true;
}

inline void rescanSensors()// TODO NEEDS DEBUG_MODE/SAFE_MODE
{

}

inline void processClientCommands()// TODO NEEDS DEBUG_MODE/SAFE_MODE
{

}

inline void flushSerialOutput()// TODO NEEDS DEBUG_MODE/SAFE_MODE
{

}

// This function gets called once when the program first starts; it initializes the serial connection, and sets up
// various registers and states of the Arduino for the program.
void setup()
{
    // Start a serial connection.
    Serial.begin(BAUD_RATE);

  #ifdef DEBUG_MODE
  {
    // Send a copy of all the registry values we alter before setup starts.
    const byte values[] = {RESPONSE_PREFIX | COMMAND_DEBUG_LOG, DEBUG_SETUP_START, 0, 0, 0, 0,
                           DEBUG_VALUE_DDRB, DDRB, DEBUG_VALUE_DDRC, DDRC, DEBUG_VALUE_DDRD, DDRD,
                           DEBUG_VALUE_PORTB, PORTB, DEBUG_VALUE_PORTC, PORTC, DEBUG_VALUE_PORTD, PORTD,
                           DEBUG_VALUE_DIDR0, DIDR0, DEBUG_VALUE_DIDR1, DIDR1,
                           DEBUG_VALUE_ADCSRA, ADCSRA, DEBUG_VALUE_ADCSRB, ADCSRB, DEBUG_VALUE_ACSR, ACSR};
    const uint32_t time = micros();
    values[2] = time         & B11111111;
    values[3] = (time >> 8)  & B11111111;
    values[4] = (time >> 16) & B11111111;
    values[5] = (time >> 24) & B11111111;
    Serial.write(values, 28);
  }{
    // Send a copy of all the macro values the program started up with.
    const uint32_t BAUD_RATE_HOLDER = BAUD_RATE;
    const uint32_t SERIAL_BUFFER_SIZE_HOLDER = SERIAL_BUFFER_SIZE;
  #ifdef AVERAGE_COUNT
    const uint8_t AVERAGE_COUNT_HOLDER = AVERAGE_COUNT;
  #else
    const uint8_t AVERAGE_COUNT_HOLDER = 0;
  #endif
  #ifdef SAFE_MODE
    const uint8_t SAFE_MODE_HOLDER = 1;
  #else
    const uint8_t SAFE_MODE_HOLDER = 0;
  #endif
    const byte values[] = {RESPONSE_PREFIX | COMMAND_DEBUG_LOG, DEBUG_MACRO_DUMP, 0, 0, 0, 0, DEBUG_VALUE_BAUD_RATE,
                           ((BAUD_RATE_HOLDER >> 24) & B11111111), ((BAUD_RATE_HOLDER >> 16) & B11111111),
                           ((BAUD_RATE_HOLDER >> 8) & B11111111), (BAUD_RATE_HOLDER & B11111111), DEBUG_VALUE_SERIAL_SIZE,
                           ((SERIAL_BUFFER_SIZE_HOLDER >> 24) & B11111111), ((SERIAL_BUFFER_SIZE_HOLDER >> 16) & B11111111),
                           ((SERIAL_BUFFER_SIZE_HOLDER >> 8) & B11111111), (SERIAL_BUFFER_SIZE_HOLDER & B11111111),
                           DEBUG_VALUE_AVERAGE_COUNT, AVERAGE_COUNT_HOLDER, DEBUG_VALUE_DEBUG_MODE, 1,
                           DEBUG_VALUE_SAFE_MODE, SAFE_MODE_HOLDER};
    const uint32_t time = micros();
    values[2] = time         & B11111111;
    values[3] = (time >> 8)  & B11111111;
    values[4] = (time >> 16) & B11111111;
    values[5] = (time >> 24) & B11111111;
    Serial.write(values, 22)
  }
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

  #ifdef DEBUG_MODE
  {
    // Send a copy of all the registry values we alter after setup has finished.
    const byte values[] = {RESPONSE_PREFIX | COMMAND_DEBUG_LOG, DEBUG_SETUP_FINISH, 0, 0, 0, 0,
                           DEBUG_VALUE_DDRB, DDRB, DEBUG_VALUE_DDRC, DDRC, DEBUG_VALUE_DDRD, DDRD,
                           DEBUG_VALUE_PORTB, PORTB, DEBUG_VALUE_PORTC, PORTC, DEBUG_VALUE_PORTD, PORTD,
                           DEBUG_VALUE_DIDR0, DIDR0, DEBUG_VALUE_DIDR1, DIDR1,
                           DEBUG_VALUE_ADCSRA, ADCSRA, DEBUG_VALUE_ADCSRB, ADCSRB, DEBUG_VALUE_ACSR, ACSR};
                           DEBUG_VALUE_ACSR, ACSR};
    const uint32_t time = micros();
    values[2] = time         & B11111111;
    values[3] = (time >> 8)  & B11111111;
    values[4] = (time >> 16) & B11111111;
    values[5] = (time >> 24) & B11111111;
    Serial.write(values, 28);
  }
  #endif

    // Send the ready command to the client, notifying it the Arduino is ready for communcation.
    Serial.write(RESPONSE_PREFIX | DEBUG_CONNECTION_READY);
    Serial.flush();

    // Wait until an echoing ready command is received from the client.
    while(true)
    {
        while(Serial.available() == 0){}
        const byte command = Serial.read();
        if(command != (COMMAND_PREFIX | COMMAND_READY))
        {
            Serial.write(RESPONSE_PREFIX | COMMAND_READY);
            Serial.flush();
        }
    }

  #ifdef DEBUG_MODE
  {
    // Log that everything is ready and the program is about to enter the main loop.
    const byte values[] = {RESPONSE_PREFIX | COMMAND_DEBUG_LOG, DEBUG_READY, 0, 0, 0, 0};
    const uint32_t time = micros();
    values[2] = time         & B11111111;
    values[3] = (time >> 8)  & B11111111;
    values[4] = (time >> 16) & B11111111;
    values[5] = (time >> 24) & B11111111;
    Serial.write(values, 6);
  }
  #endif
}

// This gets called in a loop during program execution and handles the program's main logic.
void loop()
{
    handleAnalogInterrupt();
    handleTimerInterrupt();
    handleReadingCompletedInterrupt();


    switch(statusFlags & CURRENT_READING_TYPE)
    {
        case(READING_TYPE_NONE):
            if(processClientCommands() || flushSerialOutput() || rescanSensors())
            {
                return;
            }
        break;

        case(READING_TYPE_BATCH):
            if(processClientCommands() || flushSerialOutput())
            {
                return;
            }
        break;

        case(READING_TYPE_SINGLE):
        case(READING_TYPE_IDENTIFY):
            flushSerialOutput()
        break;

      #ifdef SAFE_MODE
        case default:
            // This should be impossible.
            reportError(ERROR_DEFAULT_READING_TYPE);
        break;
      #endif
    }
}

// Interrupt service routine that gets called by the timer at the end of every sample period.
ISR(TIMER1_COMPA_vect)
{
  #ifdef SAFE_MODE
    // Report an error if the last timer interrupt still hasn't been handled.
    if(statusFlags & TIMER_INTERRUPT_BITMASK)
    {
        reportError(ERROR_UNHANDLED_TIMER_ISR);
    }
  #endif

    // Set a flag indicating a new reading should be started.
    statusFlags |= TIMER_INTERRUPT_BITMASK;
}

// Interrupt service routine that gets called whenever an analog reading has been finished.
ISR(ANALOG_COMP_vect)
{
  #ifdef SAFE_MODE
    // Report an error if the last ADC interrupt still hasn't been handled.
    if(statusFlags & ADC_INTERRUPT_BITMASK)
    {
        reportError(ERROR_UNHANDLED_ADC_ISR);
    }
  #endif

    // Set a flag indicating an analog reading was finished.
    statusFlags |= ADC_INTERRUPT_BITMASK;
}
