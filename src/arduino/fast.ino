
#include "fast.h"





/** Sends a timestamped log message to the client.
  * @param debugCode: The debug code to send to the client, this should be one of the `DEBUG_X` constants. **/
inline void debugLog(const uint8_t debugCode)
{
    const uint32_t time = micros();
    uint8_t message[6] = {(RESPONSE_PREFIX | COMMAND_DEBUG_LOG), debugCode, 0, 0, 0, 0};
    // Write the timestamp into the message.
    message[2] = (time >> 24) & 0xff;
    message[3] = (time >> 16) & 0xff;
    message[4] = (time >> 8)  & 0xff;
    message[5] = time         & 0xff;
    // Write the message into the Arduino's serial output buffer so it'll be sent to the client.
    Serial.write(message, 6);
}


/** Sends a timestamped log message to the client followed by a dump of all the program's stack variable values.
  * @param debugCode: The debug code to send to the client, this should be one of the `DEBUG_X` constants. **/
inline void debugLogWithStack(const uint8_t debugCode, const uint8_t[] stack, const uint8_t stackLength)
{
    const uint32_t time = micros();
    uint8_t message[6] = {(RESPONSE_PREFIX | COMMAND_DEBUG_LOG), (debugCode | PAYLOAD_CONTINUATION_FLAG), 0, 0, 0, 0};
    // Write the timestamp into the message.
    message[2] = (time >> 24) & 0xff;
    message[3] = (time >> 16) & 0xff;
    message[4] = (time >> 8)  & 0xff;
    message[5] = time         & 0xff;
    // Write the message and stack variables into the Arduino's serial output buffer so it'll be sent to the client.
    Serial.write(message, 6);
    Serial.write(stack, stackLength);
}


/** Sends a timestamped log message to the client, followed by a dump of the current values of all the
  * program's heap variables and the Arduino's registers that are used within the program.
  * @param debugCode: The debug code to send to the client, this should be one of the `DEBUG_X` constants. **/
inline void debugLogDump(const uint8_t debugCode)
{
    const uint32_t time = micros();
    uint8_t message[42] = {(RESPONSE_PREFIX | COMMAND_DEBUG_LOG), (debugCode | PAYLOAD_CONTINUATION_FLAG), 0, 0, 0, 0,
        statusFlags, enabledFlags, pinModeFlags, sensorIDs[0], sensorIDs[1], sensorIDs[2], sensorIDs[3],
        ((samplePeriod >> 24) & 0xff),((samplePeriod >> 16) & 0xff),((samplePeriod >> 8) & 0xff),(samplePeriod & 0xff),
        ((BAUD_RATE >> 24) & 0xff), ((BAUD_RATE >> 16) & 0xff), ((BAUD_RATE >> 8) & 0xff), (BAUD_RATE & 0xff),
        ((dataBufferCounter >> 8) & 0xff), (dataBufferCounter & 0xff),
  #ifdef MAVERAGE_COUNT
        ((averagingBufferCounter >> 8) & 0xff), (averagingBufferCounter & 0xff),
  #endif
        ((DATA_BUFFER_SIZE >> 8) & 0xff), (DATA_BUFFER_SIZE & 0xff), AVERAGE_COUNT, CHECK_MODE, DEBUG_MODE,
        PORTB, PORTC, PORTD, DDRB, DDRC, DDRD, DIDR0, DIDR1, ADMUX, ADCSRB, ADCSRB, ACSR
    };
    // Write the timestamp into the message.
    message[2] = (time >> 24) & 0xff;
    message[3] = (time >> 16) & 0xff;
    message[4] = (time >> 8)  & 0xff;
    message[5] = time         & 0xff;
    // Write the message into the Arduino's serial output buffer so it'll be sent to the client.
    Serial.write(message, 42);
}


/** Sends a timestamped log message to the client, followed by a dump of the current values of all the
  * program's heap and stack variables and the Arduino's registers that are used within the program.
  * @param debugCode: The debug code to send to the client, this should be one of the `DEBUG_X` constants. **/
inline void debugLogDumpWithStack(const uint8_t debugCode, const uint8_t[] stack, const uint8_t stackLength)
{
    const uint32_t time = micros();
    uint8_t message[42] = {(RESPONSE_PREFIX | COMMAND_DEBUG_LOG), (debugCode | PAYLOAD_CONTINUATION_FLAG), 0, 0, 0, 0,
        statusFlags, enabledFlags, pinModeFlags, sensorIDs[0], sensorIDs[1], sensorIDs[2], sensorIDs[3],
        ((samplePeriod >> 24) & 0xff),((samplePeriod >> 16) & 0xff),((samplePeriod >> 8) & 0xff),(samplePeriod & 0xff),
        ((BAUD_RATE >> 24) & 0xff), ((BAUD_RATE >> 16) & 0xff), ((BAUD_RATE >> 8) & 0xff), (BAUD_RATE & 0xff),
        ((dataBufferCounter >> 8) & 0xff), (dataBufferCounter & 0xff),
  #ifdef MAVERAGE_COUNT
        ((averagingBufferCounter >> 8) & 0xff), (averagingBufferCounter & 0xff),
  #endif
        ((DATA_BUFFER_SIZE >> 8) & 0xff), (DATA_BUFFER_SIZE & 0xff), AVERAGE_COUNT, CHECK_MODE, DEBUG_MODE,
        PORTB, PORTC, PORTD, DDRB, DDRC, DDRD, DIDR0, DIDR1, ADMUX, ADCSRB, ADCSRB, ACSR
    };
    // Write the timestamp into the message.
    message[2] = (time >> 24) & 0xff;
    message[3] = (time >> 16) & 0xff;
    message[4] = (time >> 8)  & 0xff;
    message[5] = time         & 0xff;
    // Write the message and stack variables into the Arduino's serial output buffer so it'll be sent to the client.
    Serial.write(message, 42);
    Serial.write(stack, stackLength);
}




/** This function is called when the Arduino has reached an unrecoverable error state, like receiving a corrupted
  * command from the client. When called this function flashes the onboard LED and clears the serial input buffer.
  * It continues flashing the LED and drops any incoming serial data for the next 30 seconds before performing a
  * critical hard reset on the Arduino. **/
inline void criticalErrorMode()
{
    const uint32_t timeStart = millis();
    // Wait 30 seconds before resuming normal function.//TODO
    while((millis() - timeStart) < 30000)
    {
        // Flash the onboard LED once every 256ms.
        if((millis() >> 7) & B1)
        {
            // Turn the onboard LED off.
            PORTD &= ~B00100000;
        } else {
            // Turn the onboard LED on.
            PORTD |= B00100000;

        }
        // Discard any leftover or incoming Serial input.
        while(Serial.available())
        {
            Serial.read();
        }
    }

    // Perform a critical hard reset on the Arduino. Calling this function forcibly resets the Arduino to start
    // executing at raw memory address 0, completely ignoring any current context or safeguards. This has to be used
    // EXTREMELY CAREFULLY and running this code on any non-Arduino platform could completely brick the system.
    static void(* hardReset) (void) = 0;
    hardReset();
}


/** convenience method that tries to read the specified number of bytes into the provided buffer.
  * If the bytes can't be read due to a timeout or other cause, it reports an error and returns true.
  * @param buffer: The buffer to read the serial input into. This always starts writing at buffer offset 0.
  * @param length: The number of bytes to read from the serial input connection.
  * @returns: False if the correct number of bytes were read successfully, true otherwise. **/
inline true readSerialBytes(const uint8_t* buffer, const uint8_t length)
{
    const uint8_t count = Serial.readBytes(buffer, length);
    if(count != length)
    {
        debugLogDumpWithStack(ERROR_SERIAL_TIMEOUT, (const uint8_t[]){length, count}, 2);
        criticalErrorMode();
        return true;
    }
    return false;
}




/** Stops any currently running analog readings and resets the current reading type to NONE. **/
inline void stopAnalogReadings()
{
  #ifdef MCHECK_MODE
    // If the Arduino wasn't taking any analog readings, report an error.
    if((CURRENT_READING_TYPE_BITMASK & statusFlags) == READING_TYPE_NONE)
    {
        debugLogDump(ERROR_READING_TYPE_MISMATCH_stopAnalogReadings);
    }
  #endif

    // Disable the Analog to Digital Converter (ADC).
    ADCSRA = B00000000;
    // Set the current reading type to NONE (Same as clearing the reading type to 0).
    statusFlags &= CURRENT_READING_TYPE_BITMASK;
}




//TODO THIS FUNCTION
/** Completes a reading by either sending the data to the client, or if averaging is enabled by computing and sending
  * an average if there's been enough data points. **/
inline void completeReading()
{
  #ifdef DEBUG_MODE
    debugLog(DEBUG_COMPLETE_READING_START);
  #endif

  #ifdef MAVERAGE_COUNT
    // If enough readings have been taken to compute an average, write the average into the data buffer.
    if((averagingBufferCounter / 6) == (AVERAGE_COUNT - 1))
    {
        // TODO maybe make the times and digital readings real averages?
        // Store the time the reading was finished at.
        const uint32_t time = micros();
    //TODO TODO TODO TODO
        uint8_t digitalResults = B00000000;
        float[4] analogResults = {0f, 0f, 0f, 0f};

        for(int i = 0; i < AVERAGE_COUNT; i++)
        {
            digitalResults |= averagingBuffer[6 * i];
        }

        // TODO take the average and write into the data buffer.

    } else {
        // Otherwise just increment the averaging count by 1 full reading (6 bytes).
        averagingBufferCounter += 6;

      #ifdef DEBUG_MODE
        debugLog(DEBUG_COMPLETE_READING_END);
      #endif
      return;
    }
  #endif

    // Write the TAKE_READING response byte to the serial connection.
    Serial.write(RESPONSE_PREFIX | COMMAND_TAKE_READING);
    // Write from the data buffer into the Arduino's serial output buffer to send it to the client.
    const uint8_t memoryMode = (PIN_AX_ENABLED_BITMASK & enabledFlags)
    switch(memoryMode)
    {
        // If there's 1 pin enabled, readings take up 7 bytes.
        case(B00010000): case(B00100000): case(B01000000): case(B10000000):
          #ifdef MCHECK_MODE
            if(dataBufferCounter != 7)
            {
                debugLogDumpWithStack(ERROR_BUFFER_COUNTER_MISMATCH, (const uint8_t[]){dataBufferCounter}, 1);
            }
          #endif
            Serial.write(dataBuffer, 7);
        break;

        // If there's 2 pins enabled, readings take up 8 bytes.
        case(B00110000): case(B01010000): case(B10010000): case(B01100000): case(B10100000): case(B11000000):
          #ifdef MCHECK_MODE
            if(dataBufferCounter != 8)
            {
                debugLogDumpWithStack(ERROR_BUFFER_COUNTER_MISMATCH, (const uint8_t[]){dataBufferCounter}, 1);
            }
          #endif
            Serial.write(dataBuffer, 8);
        break;

        // If there's 3 pins enabled, readings take up 9 bytes.
        case(B01110000): case(B10110000): case(B11010000): case(B11100000):
          #ifdef MCHECK_MODE
            if(dataBufferCounter != 9)
            {
                debugLogDumpWithStack(ERROR_BUFFER_COUNTER_MISMATCH, (const uint8_t[]){dataBufferCounter}, 1);
            }
          #endif
            Serial.write(dataBuffer, 9);
        break;

        // If there's 4 pins enabled, readings take up 10 bytes.
        case(B11110000):
          #ifdef MCHECK_MODE
            if(dataBufferCounter != 10)
            {
                debugLogDumpWithStack(ERROR_BUFFER_COUNTER_MISMATCH, (const uint8_t[]){dataBufferCounter}, 1);
            }
          #endif
            Serial.write(dataBuffer, 10);
        break;

      #ifdef MCHECK_MODE
        // A reading was finished with no pins enabled.
        case(B00000000):
            debugLogDump(ERROR_EMPTY_READING_COMPLETED);
        break;
        // An illegal value was switched on, this should be impossible.
        default:
            debugLogDumpWithStack(ILLEGAL_MEMORY_MODE, (const uint8_t[]){memoryMode}, 1);
        break;
      #endif
    }

    // If the Arduino was only taking a single reading, set the current READING_TYPE back to none.
    if((CURRENT_READING_TYPE_BITMASK & statusFlags) == READING_TYPE_SINGLE)
    {
        stopAnalogReadings();
    }
    // Reset the databuffer counter.
    dataBufferCounter = 0;

  #ifdef DEBUG_MODE
    debugLog(DEBUG_COMPLETE_READING_END);
  #endif
}
// TODO THIS FUNCTION




/** Starts an analog reading on the specified analog pin.
  * @param address: The address of the pin to read from, this should be one of the `ADMUX_PIN_ADDRESS_X` constants. **/
inline void startAnalogReading(const uint8_t address)
{
  #ifdef MDEBUG_MODE
    debugLogWithStack(DEBUG_START_startAnalogReading, (const uint8_t[]){address}, 1);
  #endif

  #ifdef MCHECK_MODE
    // If the address is an unmapped pin, report an error.
    if(address > ADMUX_PIN_ADDRESS_A5)
    {
        debugLogDumpWithStack(ERROR_ILLEGAL_ANALOG_PIN_ADDRESS_startAnalogReading, (const uint8_t[]){address}, 1);
    }
    // If we started a reading on analog pin A4 (which we never use), report an error.
    if(address == ADMUX_PIN_ADDRESS_A4)
    {
        debugLogDumpWithStack(ERROR_ANALOG_PIN_A4_ACCESS_startAnalogReading, (const uint8_t[]){address}, 1);
    }
    // If the Arduino shouldn't be taking readings, report an error.
    if((CURRENT_READING_TYPE_BITMASK & statusFlags) == READING_TYPE_NONE)
    {
        debugLogDumpWithStack(ERROR_READING_TYPE_MISMATCH_startAnalogReading, (const uint8_t[]){address}, 1);
    }
  #endif

    // Sets the ADC to use the onboard 5v power rail as it's reference voltage and sets the pin address to measure.
    ADMUX = (1 << REFS0) | address;
    // Enables the ADC, starts a conversion with interrupts enabled, and sets the ADC clock to a multiplier of 16.
    ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADIE) | (1 << ADPS2);

  #ifdef MDEBUG_MODE
    debugLog(DEBUG_END_startAnalogReading);
  #endif
}


/** Handles an analog reading taken from one of the Vernier sensors by storing the current reading in a buffer and
  * starting the next reading, if applicable.
  * @param address: The address of the analog pin the reading was taken from. **/
inline void handleAnalogSensorReading(const uint8_t address)
{
  #ifdef MDEBUG_MODE
    debugLogWithStack(DEBUG_START_handleAnalogSensorReading, (const uint8_t[]){address}, 1);
  #endif

  #ifdef MCHECK_MODE
    const uint8_t readingType = CURRENT_READING_TYPE_BITMASK & statusFlags;
    // If the address is an unmapped pin, report an error.
    if(address > ADMUX_PIN_ADDRESS_A5)
    {
        debugLogDumpWithStack(ERROR_ILLEGAL_ANALOG_PIN_ADDRESS_handleAnalogSensorReading, (const uint8_t[]){address, readingType}, 2);
    }
    // If we started a reading on analog pin A4 (which we never use), report an error.
    if(address == ADMUX_PIN_ADDRESS_A4)
    {
        debugLogDumpWithStack(ERROR_ANALOG_PIN_A4_ACCESS_handleAnalogSensorReading, (const uint8_t[]){address, readingType}, 2);
    }
    // If the Arduino isn't taking a sensor value reading, report an error.
    if(!((readingType == READING_TYPE_BATCH) || (readingType == READING_TYPE_SINGLE)))
    {
        debugLogDumpWithStack(ERROR_READING_TYPE_MISMATCH_handleAnalogSensorReading, (const uint8_t[]){address, readingType}, 2);
    }
  #endif

    // Select the appropiate buffer and position variable to use to store the sensor reading in.
  #ifdef MAVERAGE_COUNT
    uint8_t* buffer = dataBuffer;
    const uint16_t pos = dataBufferCounter + 5; //The first 5 bytes contain the timestamp and digital pin readings.
  #else
    uint8_t* buffer = averagingBuffer;
    const uint16_t pos = averagingBufferCounter + 1; //The first byte contains readings from the digital pins.
  #endif

    const uint8_t memoryMode = (PIN_AX_ENABLED_BITMASK & enabledFlags) | address;
    // Store the analog reading in the buffer and start the next reading if applicable.
    switch(memoryMode)
    {
        // This is reading 1 out of 1; Store the reading in the buffer and then complete the reading.
        case(B00010000): case(B00100001): case(B01000010): case(B10000011):
            buffer[pos]   = ADCL;
            buffer[pos+1] = (ADCH & B00000011);
            completeReading();
        break;
        // This is reading 1 out of 2; Store the reading in the buffer and then start a reading on pin A1.
        case(B00110000):
        // This is reading 1 out of 3; Store the reading in the buffer and then start a reading on pin A1.
        case(B01110000): case(B10110000):
        // This is reading 1 out of 4; Store the reading in the buffer and then start a reading on pin A1.
        case(B11110000):
            buffer[pos]   = ADCL;
            buffer[pos+1] = (ADCH & B00000011);
            startAnalogReading(ADMUX_PIN_ADDRESS_A1);
        break;
        // This is reading 1 out of 2; Store the reading in the buffer and then start a reading on pin A2.
        case(B01010000): case(B01100001):
        // This is reading 1 out of 3; Store the reading in the buffer and then start a reading on pin A2.
        case(B11010000):
            buffer[pos]   = ADCL;
            buffer[pos+1] = (ADCH & B00000011);
            startAnalogReading(ADMUX_PIN_ADDRESS_A2);
        break;
        // This is reading 1 out of 2; Store the reading in the buffer and then start a reading on pin A3.
        case(B10010000): case(B10100001): case(B11000010):
        // This is reading 1 out of 3; Store the reading in the buffer and then start a reading on pin A3.
        case(B11100001):
            buffer[pos]   = ADCL;
            buffer[pos+1] = (ADCH & B00000011);
            startAnalogReading(ADMUX_PIN_ADDRESS_A3);
        break;

        // This is reading 2 out of 2; Store the reading in the buffer and then complete the reading.
        case(B00110001): case(B01010010): case(B10010011): case(B01100010): case(B10100011): case(B11000011):
            buffer[pos+1] |= ((ADCL & B00111111) << 2);
            buffer[pos+2]  = ((ADCL & B11000000) >> 6) | ((ADCH & B00000011) << 2);
            completeReading();
        break;
        // This is reading 2 out of 3; Store the reading in the buffer and then start a reading on pin A2.
        case(B01110001): case(B10110001):
        // This is reading 2 out of 4; Store the reading in the buffer and then start a reading on pin A2.
        case(B11110001):
            buffer[pos+1] |= ((ADCL & B00111111) << 2);
            buffer[pos+2]  = ((ADCL & B11000000) >> 6) | ((ADCH & B00000011) << 2);
            startAnalogReading(ADMUX_PIN_ADDRESS_A2);
        break;
        // This is reading 2 out of 3; Store the reading in the buffer and then start a reading on pin A3.
        case(B11010010): case(B11100010):
            buffer[pos+1] |= ((ADCL & B00111111) << 2);
            buffer[pos+2]  = ((ADCL & B11000000) >> 6) | ((ADCH & B00000011) << 2);
            startAnalogReading(ADMUX_PIN_ADDRESS_A3);
        break;

        // This is reading 3 out of 3; store the reading in the buffer and then complete the reading.
        case(B01110010): case(B10110011): case(B11010011): case(B11100011):
            buffer[pos+2] |= ((ADCL & B00001111) << 4);
            buffer[pos+3]  = ((ADCL & B11110000) >> 4) | ((ADCH & B00000011) << 4);
            completeReading();
        break;
        // This is reading 3 out of 4; Store the reading in the buffer and then start a reading on pin 3.
        case(B11110010):
            buffer[pos+2] |= ((ADCL & B00001111) << 4);
            buffer[pos+3]  = ((ADCL & B11110000) >> 4) | ((ADCH & B00000011) << 4);
            startAnalogReading(ADMUX_PIN_ADDRESS_A3);
        break;

        // This is reading 4 out of 4; Store the reading in the buffer and then complete the reading.
        case(B11110011):
            buffer[pos+3] |= ((ADCL & B00000011) << 6);
            buffer[pos+4]  = ((ADCL & B11111100) >> 2) | ((ADCH & B00000011) << 6);
            completeReading();
        break;

      #ifdef MCHECK_MODE
        // These cases cover the possibility that an interrupt was received for a disabled pin.
        case(B00000000): case(B00000001): case(B00000010): case(B00000011): // No analog pins are enabled.
                         case(B00010001): case(B00010010): case(B00010011): // Analog pin 0 is enabled.
        case(B00100000):                  case(B00100010): case(B00100011): // Analog pin 1 is enabled.
        case(B01000000): case(B01000001):                  case(B01000011): // Analog pin 2 is enabled.
        case(B10000000): case(B10000001): case(B10000010):                  // Analog pin 3 is enabled.
                                          case(B00110010): case(B00110011): // Analog pins 1,2 are enabled.
                        case(B01010001):                   case(B01010011): // Analog pins 1,3 are enabled.
                        case(B10010001):  case(B10010010):                  // Analog pins 1,4 are enabled.
        case(B01100000):                                   case(B01100011): // Analog pins 2,3 are enabled.
        case(B10100000):                  case(B10100010):                  // Analog pins 2,4 are enabled.
        case(B11000000): case(B11000001):                                   // Analog pins 3,4 are enabled.
                                                           case(B01110011): // Analog pins 1,2,3 are enabled.
                                          case(B10110010):                  // Analog pins 1,2,4 are enabled.
                         case(B11010001):                                   // Analog pins 1,3,4 are enabled.
        case(B11100000):                                                    // Analog pins 2,3,4 are enabled.
            debugLogDumpWithStack(ERROR_DISABLED_PIN_INTERRUPTED_handleAnalogSensorReading, (const uint8_t[]){memoryMode}, 1);
        break;

        // None of the cases matched, so an impossible value was switched on.
        default:
            debugLogDumpWithStack(ERROR_ILLEGAL_INTERRUPT_handleAnalogSensorReading, (const uint8_t[]){memoryMode}, 1);
        break;
      #endif
    }

  #ifdef MDEBUG_MODE
    debugLog(DEBUG_END_handleAnalogSensorReading);
  #endif
}


/** Handles a reading taken from the ID line of a Vernier sensor by checking for a change. If a sensor changed the
  * client is sent a response notifying it.
  * If there's more ports to scan afterwards, this starts a scan on the next one. **/
inline void handleAnalogSensorIDReading()
{
  #ifdef MDEBUG_MODE
    debugLog(DEBUG_START_handleAnalogSensorIDReading);
  #endif

    const uint8_t address = (MUX_ADDRESS_BITMASK & statusFlags);

  #ifdef MCHECK_MODE
    const uint8_t readingType = CURRENT_READING_TYPE_BITMASK & statusFlags;
    // If the Arduino isn't taking a sensor ID reading, report an error.
    if(!(readingType == READING_TYPE_IDENTIFY))
    {
        debugLogDumpWithStack(ERROR_READING_TYPE_MISMATCH_handleAnalogSensorIDReading, (const uint8_t[]){readingType}, 1);
    }
    // If the address is of an unmapped port, report an error.
    if(!((address == MUX_PORT_ADDRESS_ANALOG_1) || (address == MUX_PORT_ADDRESS_ANALOG_2) ||
       (address == MUX_PORT_ADDRESS_DIGITAL_1) || (address == MUX_PORT_ADDRESS_DIGITAL_1)))
    {
        debugLogDumpWithStack(ERROR_ILLEGAL_PORT_ADDRESS_handleAnalogSensorIDReading, (const uint8_t[]){address}, 1);
    }
  #endif

    // Store the old sensor ID so we can check if it's changed.
    const uint8_t oldSensorID = sensorIDs[address >> 2];
    // Identify the sensor connected to this address, if there are any.
    identiftyVernierSensor(address);
    // If the sensor was changed, notify the client of the new sensor's ID.
    if(oldSensorID != sensorIDs[address >> 2])
    {
        Serial.write((const uint8_t[]){(RESPONSE_PREFIX | COMMAND_SCAN_SENSORS), address, sensorIDs[address >> 2]}, 3);
    }

    // Switch on the address of the pin that was just identified to know what port to check next.
    switch(address)
    {
        case(MUX_PORT_ADDRESS_ANALOG_1):
            // Connect analog port 2 to the Vernier interface's MUX so it can be identified next.
            PORTD = (PORTD & ~MUX_ADDRESS_BITMASK) | MUX_PORT_ADDRESS_ANALOG_2;
            // Start an analog reading on analog pin A5; This pin is connected to the sensor's ID line.
            startAnalogReading(ADMUX_PIN_ADDRESS_A5);
        break;
        case(MUX_PORT_ADDRESS_ANALOG_2):
            // Connect digital port 1 to the Vernier interface's MUX so it can be identified next.
            PORTD = (PORTD & ~MUX_ADDRESS_BITMASK) | MUX_PORT_ADDRESS_DIGITAL_1;
            // Start an analog reading on analog pin A5; This pin is connected to the sensor's ID line.
            startAnalogReading(ADMUX_PIN_ADDRESS_A5);
        break;
        case(MUX_PORT_ADDRESS_DIGITAL_1):
            // Connect digital port 2 to the Vernier interface's MUX so it can be identified next.
            PORTD = (PORTD & ~MUX_ADDRESS_BITMASK) | MUX_PORT_ADDRESS_DIGITAL_2;
            // Start an analog reading on analog pin A5; This pin is connected to the sensor's ID line.
            startAnalogReading(ADMUX_PIN_ADDRESS_A5);
        break;
        case(MUX_PORT_ADDRESS_DIGITAL_2):
            // There's no more sensors to read from, so stop analog readings.
            stopAnalogReadings();
        break;
      #ifdef MCHECK_MODE
        default:
            debugLogDumpWithStack(ERROR_ILLEGAL_READING_ADDRESS_handleAnalogSensorIDReading, (const uint8_t[]){address}, 1);
        break;
      #endif
    }

  #ifdef MDEBUG_MODE
    debugLog(DEBUG_END_handleAnalogSensorIDReading);
  #endif
}


/** Checks if there's an unhandled interrupt from the ADC, and if there is this method handles it. **/
inline void checkADCInterrupt()
{
    // Report an error if an ADC interrupt was missed and unhandled.
    if(ADC_INTERRUPT_MISSED_BITMASK & statusFlags)
    {
        debugLogDump(ERROR_UNHANDLED_ISR_checkADCInterrupt);
    }

    // If there is an ADC interrupt to handle, handle it.
    if(ADC_INTERRUPT_SIGNAL_BITMASK & statusFlags)
    {
      #ifdef MDEBUG_MODE
        debugLog(DEBUG_START_checkADCInterrupt);
      #endif

        // Get the address of the analog pin that the reading was taken from.
        const uint8_t address = (ADMUX_ADDRESS_BITMASK & ADMUX);
        // If the reading was taken from pins A0,A1,A2,A3 handle it as a sensor reading.
        if(address <= ADMUX_PIN_ADDRESS_A3)
        {
            handleAnalogSensorReading(address);
        } else
        // If the reading was taken from pin A5 handle it as a sensor ID reading.
        if(address == ADMUX_PIN_ADDRESS_A5)
        {
            handleAnalogSensorIDReading();
        }
      #ifdef MCHECK_MODE
        // If the reading was taken from pin A4, report an error.
        else if(address == ADMUX_PIN_ADDRESS_A4)
        {
            debugLogDumpWithStack(ERROR_ANALOG_PIN_A4_ACCESS_checkADCInterrupt, (const uint8_t[]){address}, 1);
        }
        // If the reading was taken from an unmapped pin, report an error. This should be impossible.
        else {
            debugLogDumpWithStack(ERROR_ILLEGAL_ANALOG_PIN_ADDRESS_checkADCInterrupt, (const uint8_t[]){address}, 1);
        }
      #endif

        // Clear the ADC interrupt flag.
        statusFlags &= ~ADC_INTERRUPT_SIGNAL_BITMASK;

      #ifdef MDEBUG_MODE
        debugLog(DEBIG_END_checkADCInterrupt);
      #endif
    }
  #ifdef MDEBUG_MODE
    // Log that ADC interrupts were checked for, but there weren't any to handle.
    else {
        debugLog(DEBUG_NONE_checkADCInterrupt);
    }
  #endif
}




/** Starts a new sensor reading by clearing out any old reading data, recording a timestamp for when the reading
  * started and the current values of the digital pins before starting any necessary analog readings. **/
inline void startNewSensorReading()
{
  #ifdef MDEBUG_MODE
    debugLog(DEBUG_START_startNewSensorReading);
  #endif
  #ifdef MCHECK_MODE
    const uint8_t readingType = CURRENT_READING_TYPE_BITMASK & statusFlags;
    // Report an error if the Arduino shouldn't be taking readings.
    if(!((readingType == READING_TYPE_SINGLE) || readingType == READING_TYPE_BATCH))
    {
        debugLogDumpWithStack(ERROR_READING_TYPE_MISMATCH_startNewSensorReading, (const uint8_t[]){readingType}, 1);
    }
  #endif

    // Select the appropiate buffer and position variables to store the sensor reading in.
  #ifdef MAVERAGE_COUNT
    uint8_t* buffer = dataBuffer;
    const uint8_t pos = dataBufferCounter + 4; //The first 4 bytes contain the timestamp.
  #else
    uint8_t* buffer = averagingBuffer;
    const uint8_t pos = averagingBufferCounter;
  #endif

    // Clear any leftover data from the buffer
    switch(PIN_AX_ENABLED_BITMASK & enabledFlags)
    {
        // If there aren't any analog pins enabled, do nothing.
        case B00000000:
        break;
        // With 1 analog pin enabled, each measurement takes up 2 bytes, so only clear the next 3 bytes.
        case B00010000: case B00100000: case B01000000: case B10000000:
            buffer[pos] = 0; buffer[pos+1] = 0; buffer[pos+2] = 0;
        break;
        // With 2 analog pins enabled, each measurement takes up 4 bytes, so only clear the next 4 bytes.
        case B00110000: case B01010000: case B10010000: case B01100000: case B10100000: case B11000000:
            buffer[pos] = 0; buffer[pos+1] = 0; buffer[pos+2] = 0; buffer[pos+3] = 0;
        break;
        // With 3 analog pins enabled, each measurement takes up 5 bytes, so only clear the next 5 bytes.
        case B01110000: case B10110000: case B11010000: case B11100000:
            buffer[pos] = 0; buffer[pos+1] = 0; buffer[pos+2] = 0; buffer[pos+3] = 0; buffer[pos+4] = 0;
        break;
        // With 4 analog pins enabled, each measurement takes up 6 bytes, so only clear the next 6 bytes.
        case B11110000:
            buffer[pos] = 0; buffer[pos+1] = 0; buffer[pos+2] = 0; buffer[pos+3] = 0; buffer[pos+4] = 0; buffer[pos+5] = 0;
        break;
      #ifdef MCHECK_MODE
        // None of the cases matched, so an impossible value was switched on.
        default:
            debugLogDump(ERROR_ILLEGAL_ENABLED_PINS_startNewSensorReading);
        break;
      #endif
    }

    // Store the time that the reading was started at in the data buffer.
    const uint32_t time = micros();
    dataBuffer[dataCounter]   = (time >> 24) & 0xff;
    dataBuffer[dataCounter+1] = (time >> 16) & 0xff;
    dataBuffer[dataCounter+2] = (time >> 8)  & 0xff;
    dataBuffer[dataCounter+3] = time         & 0xff;

    // Record the values of any digital pins that are enabled and in input mode.
    if(PORT_DIGITAL_1_ENABLED_BITMASK & enabledFlags)
    {
        // Store the values of pins 2,3,4,5.
        buffer[pos] = ((PORTB & B00111100) >> 2) & pinModeFlags;
    }
    if(PORT_DIGITAL_2_ENABLED_BITMASK & enabledFlags)
    {
        // Store the values of pins 6,7,8,9.
        buffer[pos] |= ((PORTB & B11000000) >> 2) | ((PORTD & B00000011) << 6) & pinModeFlags;
    }

    // Start an analog reading on the lowest address analog pin first (in any are enabled).
    if(PIN_A0_ENABLED_BITMASK & enabledFlags)
    {
        startAnalogReading(ADMUX_PIN_ADDRESS_A0);
    } else
    if(PIN_A1_ENABLED_BITMASK & enabledFlags)
    {
        startAnalogReading(ADMUX_PIN_ADDRESS_A1);
    } else
    if(PIN_A2_ENABLED_BITMASK & enabledFlags)
    {
        startAnalogReading(ADMUX_PIN_ADDRESS_A2);
    } else
    if(PIN_A3_ENABLED_BITMASK * enabledFlags)
    {
        startAnalogReading(ADMUX_PIN_ADDRESS_A3);
    } else {
        // Complete the reading if there's no analog pins to take readings from.
        completeReading();
    }

  #ifdef MDEBUG_MODE
    debugLog(DEBUG_END_startNewSensorReading);
  #endif
}


/** Checks if there's an unhandled interrupt from the ADC, and if there is this method handles it. **/
inline void checkTimerInterrupt()
{
    // Report an error if a TIMER interrupt was missed and unhandled.
    if(TIMER_INTERRUPT_MISSED_BITMASK & statusFlags)
    {
        debugLogDump(ERROR_UNHANDLED_ISR_checkTimerInterrupt);
    }

    // If there's a timer interrupt to handle, handle it.
    if(TIMER_INTERRUPT_SIGNAL_BITMASK & statusFlags)
    {
      #ifdef MDEBUG_MODE
        debugLog(DEBUG_START_checkTimerInterrupt);
      #endif

        startNewSensorReading();

        // Clear the TIMER interrupt flag.
        statusFlags &= ~TIMER_INTERRUPT_SIGNAL_BITMASK;

      #ifdef MDEBUG_MODE
        debugLog(DEBUG_END_checkTimerInterrupt);
      #endif
    }
  #ifdef MDEBUG_MODE
    // Log that TIMER interrupts were checked for, but there weren't any to handle.
    else {
        debugLog(DEBUG_NONE_checkTimerInterrupt);
    }
  #endif
}




/** Sets the Arduino's sampling period if it's different from the current one by changing the TIMER1 prescalar and
  * compare match value. This doesn't change the clock or counter.**/
inline void setSamplePeriod()
{
    // Read the sample period that the client is requesting (always given in ns).
    uint8_t data[4];
    if(readSerialBytes(data, 4))
    {
        return;
    }
    uint32_t requestedPeriod =
        ((uint32_t)data[3] << 24) | (uint32_t)(data[2] << 16) | (uint32_t)(data[1] << 8) | (uint32_t)data[0];
    // Compute the number of clock cycles it corresponds to. (1 clock cycle is 250ns).
    requestedPeriod /= 250;
    // Periods less than 16 clock cycles aren't supported due to potential instabilities.
    if(requestedPeriod < 16)
    {
        debugLog(ERROR_SAMPLE_PERIOD_TOO_SMALL_processClientCommands);
        break;
    }
    // If the requested period is already set as the sample period, return.
    if(requestedPeriod == samplePeriod)
    {
        return;
    }

    // Set TIMER1's clock prescalar based on the highest power of two the period is cleanly divisble by, and how many
    // clock cycles will need to be counted. The TIMER1 clock can only count up to 65535, so for instance, to count
    // higher than 65535, it needs a prescalar higher than 1 (8 is the next highest).

    // If the clock cycle count is greater than (65535 * 256), or divisble by 1024.
    if((requestedPeriod > 16776960) || (requestedPeriod & (uint16_t)B1111111111) == 0)
    {
        // This sets the prescalar to 1024 and divides the requested cycle count by 1024.
        TCCR1B = (1 << CS12) | (1 << CS10);
        requestedPeriod >> 10;
        // Set the new sample period.
        samplePeriod = (uint32_t)requestedPeriod * 250 * 1024;
    } else
    // If the clock cycle count is greater than (65535 * 64), or divisble by 256.
    if((requestedPeriod > 4194240) || ((requestedPeriod & B11111111) == 0))
    {
        // This sets the prescalar to 256 and divides the requested cycle count by 256.
        TCCR1B = (1 << CS12);
        requestedPeriod >> 8;
        samplePeriod = (uint32_t)requestedPeriod * 250 * 256;
    } else
    // If the clock cycle count is greater than (65535 * 8), or divisble by 64.
    if((requestedPeriod > 524280) || ((requestedPeriod & B111111) == 0))
    {
        // This sets the prescalar to 64 and divides the requested cycle count by 64.
        TCCR1B = (1 << CS11) | (1 << CS10);
        requestedPeriod >> 6;
        samplePeriod = (uint32_t)requestedPeriod * 250 * 64;
    } else
    // If the clock cycle count is greater than (65535 * 1), or divisble by 8.
    if((requestedPeriod > 65535) || ((requestedPeriod & B111) == 0))
    {
        // This sets the prescalar to 8 and divides the requested cycle count by 8.
        TCCR1B = (1 << CS11);
        requestedPeriod >> 3;
        samplePeriod = (uint32_t)requestedPeriod * 250 * 8;
    } else {
        // The prescalar can be 1. This sets the prescalar to 1.
        TCCR1B = (1 << CS10);
        samplePeriod = (uint32_t)requestedPeriod * 250;
    }

    // Set the number of prescalar adjusted clock cycles TIMER1 should trigger after.
    OCR1AH = (requestedPeriod >> 8) & 0xff;
    OCR1AL = requestedPeriod & 0xff;
}


inline void processClientCommands()
{
    // Return immediately if there's no commands to process.
    if(!Serial.available())
    {
        return;
    }

    const uint8_t command = Serial.read();
    // If the first byte received isn't a command byte, report an error.
    if((command & B11110000) != COMMAND_PREFIX)
    {
        debugLogDumpWithStack(ERROR_ILLEGAL_COMMAND_CODE_processClientCommands, (const uint8_t[]){command}, 1);
        criticalErrorMode();
        return;
    }

    // Switch on just the command code without the COMMAND_PREFIX.
    switch(command & B00001111)
    {
        case(COMMAND_SET_SAMPLE_RATE):
            setSamplePeriod();
            // Note that we don't break here, so we print the actual sample period back to the client.
        case(COMMAND_GET_SAMPLE_RATE):
            // Send the current sample period to the client.
            Serial.write(const uint8_t[]{(RESPONSE_PREFIX | COMMAND_GET_SAMPLE_RATE), ((samplePeriod >> 24) & 0xff),
                            ((samplePeriod >> 16) & 0xff), ((samplePeriod >> 8) & 0xff), (samplePeriod & 0xff)}, 5);
        break;

        case(COMMAND_SET_PORT_STATES):
            // Read in a payload containing the new port states in a bit array.
            uint8_t data[1];
            readSerialBytes(data, 1);
            // Set the new port states. The first 4 bits of the payload contain a bit representing whether or not
            // a port should be enabled, and the last 4 bits are a mask of which ports should be changed.
            const uint8_t mask = (data[0] >> 4) & B00001111;
            enabledFlags = (enabledFlags & ~mask) | (data[0] & mask);
        break;
        case(COMMAND_GET_PORT_STATES):
            // Send the current port states to the client.
            Serial.write((const uint8_t[]){(RESPONSE_PREFIX | COMMAND_GET_PORT_STATES),
                                           (PORT_X_ENABLED_BITMASK & enabledFlags)}, 2);
        break;

        case(COMMAND_SET_PIN_STATES):
            // Read in a payload containing the new pin states in a bit array.
            uint8_t data[3];
            readSerialBytes(data, 3);
            // Set the new pin states. The first 4 bits of the payload represent whether or not an analog pin should
            // be enabled, and the next 4 bits are a mask for which analog pins should be changed. The next 8 bits
            // containg the pin mode for each digital pin the Vernier interface uses, and the last 8 bits are a mask
            // for which digital pins should be updated.
        break;
        case(COMMAND_GET_PIN_STATES):
            // Send the current pin states to the client.
            Serial.write((const uint8_t[]){(RESPONSE_PREFIX | COMMAND_GET_PIN_STATES, pinModeFlags, 
                                           (PIN_AX_ENABLED_BITMASK & enabledFlags))}, 3);
        break;

        case(COMMAND_GET_SENSOR_IDS):

        break;
        case(COMMAND_SET_SENSOR_IDS):

        break;
        case(COMMAND_TAKE_READING):

        break;
        case(COMMAND_START_BATCH_READING):

        break;
        case(COMMAND_STOP_BATCH_READING):

        break;
        case(COMMAND_SCAN_SENSORS):

        break;
      #ifdef MCHECK_MODE
        case(COMMAND_DEBUG_LOG):

        break;
        case(COMMAND_READY):

        break;
        default:

        break;
      #endif
    }
}




void setup()
{
// Make sure pins 10 and 11 are output!
}

/** This gets called in a loop during program execution and handles the program's main logic. **/
void loop()
{
    // First check for any unhandled interrupts.
    checkADCInterrupt(); checkTimerInterrupt();

    // The behavior of the main loop depends on whether a reading is being taken, and what kind of reading it is.
    switch(CURRENT_READING_TYPE_BITMASK & statusFlags)
    {
        case(READING_TYPE_NONE):
            processClientCommands(false);
            // Start a sensor rescanning and flush any leftover serial output, since nothing time-critical is running.
            rescanSensors();
            Serial.flush();
        break;
        case(READING_TYPE_IDENTIFY):
            processClientCommands(true);
            // Flush any leftover serial output, since nothing time-critical is running.
            Serial.flush();
        break;
        case(READING_TYPE_SINGLE): case(READING_TYPE_BATCH):
            processClientCommands(true);
        break;
      #ifdef MCHECK_MODE
        default:
            debugLogDump(ERROR_ILLEGAL_READING_TYPE_loop, (const uint8_t[]){CURRENT_READING_TYPE_BITMASK & statusFlags}, 1)
        break;
      #endif
    }
}



/** Interrupt service routine that gets called by the timer at the end of every sample period. **/
ISR(TIMER1_COMPA_vect)
{
    // Set the interrupt missed flag if the last timer interrupt still hasn't been handled.
    if(ADC_INTERRUPT_SIGNAL_BITMASK & statusFlags)
    {
        statusFlags |= TIMER_INTERRUPT_MISSED_BITMASK;
    }
    // Set the ADC interrupt flag to indicate a new reading should be started.
    statusFlags |= ADC_INTERRUPT_SIGNAL_BITMASK;
}

/** Interrupt service routine that gets called by the ADC when a reading has been finished. **/
ISR(ANALOG_COMP_vect)
{
    // Set the interrupt missed flag if the last ADC interrupt still hasn't been handled.
    if(TIMER_INTERRUPT_SIGNAL_BITMASK & statusFlags)
    {
        statusFlags |= ADC_INTERRUPT_MISSED_BITMASK;
    }
    // Set the TIMER interrupt flag to indicate an analog reading was finished.
    statusFlags |= TIMER_INTERRUPT_SIGNAL_BITMASK;
}

























// This function gets called once when the program first starts; it initializes the serial connection, and sets up
// various registers and states of the Arduino for the program.
void setup()
{
    // Start a serial connection.
    Serial.begin(BAUD_RATE);
    // Wait for the Serial port to be intialized.
    while(!Serial){}

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

    // Disable PWM output from TIMER1 and set it to run in Clear on Timer Compare (CTC) mode.
    TCCR1A = B00000000;
    // Stop the counter by setting it's prescalar to 0.
    TCCR1B = B00000000;

    // Disable digital readings on analog pins A0,A1,A2,A3,A4,A5, without changing bits 6 and 7.
    DIDR0 |= (1 << ADC0D) | (1 << ADC1D) | (1 << ADC2D) | (1 << ADC3D) | (1 << ADC4D) | (1 << ADC5D);
    // Disable analog comparisons on digital pins.
    DIDR1 |= (1 << AIN0D) | (1 << AIN1D);
    // Ensure the Analog to Digital Converter is enabled and set it to use a clock prescalar of 16.
    ADCSRA |= (1 << ADEN) | (1 << ADPS2);
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


//==== LIMITATIONS AND CONCERNS ====//
/**
 Baud rates have to be between 0 and 2^32 (512 MBps)
 Data buffer sizes have to be between 0 and 65536 (64KB)
 Average count has to be between 0 and 256

 Averages on digital sensors are 1 if the sensor was 1 at any point during any of the readings, and only 0 if it was always 0. This isn't a real average.
 The average is taken between the start time and end time, not the average of every time-point used.

 Digital Sensor 2 takes a little longer to read from than digital sensor 1.
 Analog sensors are read in order one at a time, so higher address analog pins will be read after others if other pins are enabled.

 There are 2 command codes still unused (13 and 15)
 There are 2 bits unused in statusFlags (11111100)
**/

/**
 Things to enforce later:
    debugLogs must take 'DEBUG_'
    debugLogDumps must take 'ERROR_'
**/

// TODO SO WHATS THE POINT OF THE DATA BUFFER NOW THEN?
