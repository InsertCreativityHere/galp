
#include "fast.h"

/** This function gets called once when the program first starts. It initializes the serial connection and configures
  * various registers that the program uses. **/
void setup()
{
    establishSerialConnection();

  #ifdef MDEBUG_MODE
    debugDump(INFO_START_SETUP);
  #endif

    // Set analog pins A0,A1,A2,A3,A4,A5 to input mode without changing bit 6 or 7.
    DDRC &= B11000000;
    // Set pins 2,3,4,5,6,7,8,9,12 to input mode and pins 10,11,13 to output mode without changing pins 0,1,14,15.
    DDRB = (DDRB & B11000000) | B00010011;
    DDRD |= B11111100;

    // Disable pullup resistors (and set the output value to 0) on pins 2,3,4,5,6,7,8,9,10,11,13 and enable the pullup
    // resistor on pin 12 (the on-board push button).
    PORTB = (PORTB & B11000000) | B00010000;
    PORTD &= B00000011;

    // Disable PWM output from TIMER1 and set it to run in Clear on Timer Compare (CTC) mode.
    TCCR1A = B00000000;
    // Stop the counter by setting it's prescalar to 0.
    TCCR1B = B00000000;
    // Disable TIMER1 interrupts.
    TIMSK1 = B00000000;

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

  #ifdef MDEBUG_MODE
    debugLog(INFO_END_SETUP);
  #endif

    // Respond with a SERIAL_READY and ensure that it was sent properly.
    if(Serial.write(COMMAND_SOURCE_ARDUINO | COMMAND_SERIAL_ACCEPT) != 1)
    {
        // Continually flash the onboard LED to indicate an error.
        while(true)
        {
            // Toggle the onboard LED once every 256ms.
            if(millis() & B10000000)
            {
                PORTB ^= B00100000;
            }
        }
    }
    // Turn the onboard LED off now that the serial port is ready.
    PORTB &= ~B00100000;
}



/** This function gets called in a loop by the Arduino during execution. It contains the main program logic for
  * handling client requests and all the readings/operations the Arduino is currently running. **/
void loop()
{
  #ifdef MDEBUG_MODE
    debugLog(INFO_START_LOOP);
  #endif

    // The behavior of the main loop depends on whether a reading is being taken, and what kind of reading it is.
    switch(statusFlags & CURRENT_READING_TYPE_BITMASK)
    {
        // Nothing is currently running, so rescan the sensor ports for changes and flush the serial output.
        case(READING_TYPE_NONE):
            scanSensors();
            Serial.flush();
        break;

        // The Arduino is taking a single sensor reading, so only check for ADC interrupts (which indicate the
        // Arduino has finished reading a single analog pin).
        case(READING_TYPE_SINGLE):
            checkADCInterrupt();
        break;

        // The Arduino is currently in the middle of a batch reading, so we check for ADC and TIMER1 interrupts.
        case(READING_TYPE_BATCH_WAITING): case(READING_TYPE_BATCH_RUNNING):
            checkADCInterrupt();
            checkTIMER1Interrupt();
        break;

        // The Arduino is currently in the middle of a poll reading, so only poll the digital pins.
        case(READING_TYPE_POLLING):
          pollDigitalPins();
        break;

        // The Vernier interface ports are being rescanned, this isn't time-sensative so flush the serial output.
        case(READING_TYPE_IDENTIFY):
            Serial.flush();
        break;

      #ifdef MCHECK_MODE
        default:
            debugDump(ERROR_ILLEGAL_READING_TYPE_loop);
        break;
      #endif
    }

    // Process the next client command that was sent, if any.
    processClientCommands();
}



/** Interrupt service routine that gets called by TIMER1 at the end of every sample period during batch readings. **/
ISR(TIMER1_COMPA_vect)
{
  #ifdef MCHECK_MODE
    // Set the TIMER_INTERRUPT_MISSED flag if the previous timer interrupt still hasn't been handled.
    if(statusFlags & TIMER_INTERRUPT_SIGNAL_BITMASK)
    {
        statusFlags |= TIMER_INTERRUPT_MISSED_BITMASK;
    }
  #endif
    // Set the timer interrupt flag to indicate a new reading should be started.
    statusFlags |= TIMER_INTERRUPT_SIGNAL_BITMASK;
}



/** Interrupt service routine that gets called by the ADC when an analog pin reading has been finished. **/
ISR(ANALOG_COMP_vect)
{
    // Set the ADC_INTERRUPT_MISSED flag if the previous ADC interrupt still hasn't been handled.
    if(statusFlags & ADC_INTERRUPT_SIGNAL_BITMASK)
    {
        statusFlags |= ADC_INTERRUPT_MISSED_BITMASK;
    }
    // Set the ADC interrupt flag to indicate an analog reading was finished.
    statusFlags |= ADC_INTERRUPT_SIGNAL_BITMASK;
}



inline void processClientCommands()
{
  #ifdef MDEBUG_MODE
    debugLog(INFO_START_processClientCommands);
  #endif

    // If there's no commands to process, do nothing.
    if(!Serial.available())
    {
      #ifdef MDEBUG_MODE
        debugLog(INFO_NONE_processClientCommands);
      #endif
        return;
    }

    const uint8_t command = Serial.read();
    // If the first byte isn't a command byte from the Arduino, report an error and enter serial panic mode.
    if((command & COMMAND_SOURCE_BITMASK) != COMMAND_SOURCE_CLIENT)
    {
        debugDumpWithStack(ERROR_ILLEGAL_COMMAND_SOURCE_processClientCommands, {command});
        serialPanicMode();
        return;
    }

    // Switch on the received command code (ignoring the source prefix).
    switch(command & COMMAND_CODE_BITMASK)
    {
        case(COMMAND_SET_SAMPLE_PERIOD):   setSamplePeriod();
        case(COMMAND_GET_SAMPLE_PERIOD):   getSamplePeriod(); break;
        case(COMMAND_SET_PORT_STATES):     setPortStates();
        case(COMMAND_GET_PORT_STATES):     getPortStates(); break;
        case(COMMAND_SET_APIN_STATES):     setAnalogPinStates();
        case(COMMAND_GET_APIN_STATES):     getAnalogPinStates(); break;
        case(COMMAND_SET_DPIN_MODES):      setDigitalPinModes();
        case(COMMAND_GET_DPIN_MODES):      getDigitalPinModes(); break;
        case(COMMAND_SET_DPIN_OUTPUTS):    setDigitalPinOutputs();
        case(COMMAND_GET_DPIN_OUTPUTS):    getDigitalPinOutputs(); break;
        case(COMMAND_SET_SENSOR_ID):       setSensorID();
        case(COMMAND_GET_SENSOR_IDS):      getSensorIDs(); break;
        case(COMMAND_TAKE_SINGLE_READING): takeSingleReading(); break;
        case(COMMAND_START_BATCH_READING): startBatchReading(); break;
        case(COMMAND_START_POLL_READING):  startPollReading(); break;
        case(COMMAND_SCAN_SENSORS):        scanSensors(); break;
        case(COMMAND_STOP_READING):        stopReading(); break;

        case(COMMAND_SERIAL_PANIC): case(COMMAND_SERIAL_BROADCAST):
        case(COMMAND_SERIAL_ACCEPT): case(COMMAND_SERIAL_READY):
            // The serial connection needs to be re-established.
            serialPanicMode();
        break;

      #ifdef MCHECK_MODE
        case(COMMAND_DEBUG_LOG):
            debugDump(ERROR_DEBUG_LOG_processClientCommands); break;
        default:
            debugDumpWithStack(ERROR_ILLEGAL_COMMAND_processClientCommands, {command}); break;
      #endif
    }

  #ifdef MDEBUG_MODE
    debugLog(INFO_END_processClientCommands);
  #endif
}



inline void getSamplePeriod()
{
  #ifdef MDEBUG_MODE
    debugLog(INFO_START_getSamplePeriod);
  #endif

    // Write the sample period into the serial output buffer.
    writeSerialBytes({(uint8_t)(COMMAND_SOURCE_ARDUINO | COMMAND_GET_SAMPLE_PERIOD),
                      (uint8_t)((samplePeriod >> 24) & 0xff),
                      (uint8_t)((samplePeriod >> 16) & 0xff),
                      (uint8_t)((samplePeriod >> 8)  & 0xff),
                      (uint8_t) (samplePeriod        & 0xff)});

  #ifdef MDEBUG_MODE
    debugLog(INFO_END_getSamplePeriod);
  #endif
}



inline void setSamplePeriod()
{
  #ifdef MDEBUG_MODE
    debugLog(INFO_START_setSamplePeriod);
  #endif

    // Read the requested sample period from the serial input buffer.
    uint8_t data[4];
    readSerialBytes(data);
    uint32_t requestedPeriod = ((uint32_t)data[0] << 24) |
                               ((uint32_t)data[1] << 16) |
                               ((uint32_t)data[2] <<  8) |
                                (uint32_t)data[1];

    // If the Arduino is already set with the requested sample period, do nothing.
    if(samplePeriod == requestedPeriod)
    {
        return;
    }
    // Compute the number of clock cycles it corresponds to (1 clock cycle is 250ns).
    requestedPeriod /= 250;
    // Periods less than 16 clock cycles aren't supposed for stability reasons.
    if(requestedPeriod < 16)
    {
        debugLogWithStack(ERROR_UNSTABLE_SAMPLE_PERIOD_setSamplePeriod, data);
        return;
    }

    // Set TIMER1's clock prescalar based on the highest power of two the period is cleanly divisble by, and how
    // many clock cycles will need to be counted. The TIMER1 clock can only count up to 65535, so for instance, to
    // count higher than 65535, it needs a prescalar higher than 1 (8 is the next highest).

    // If the clock cycle count is greater than (65535 * 256), or divisble by 1024.
    if((requestedPeriod > 16776960) || (requestedPeriod & (uint16_t)1023) == 0)
    {
        // This sets the prescalar to 1024 and divides the requested cycle count by 1024.
        TCCR1B = (1 << CS12) | (1 << CS10) | (1 << WGM12);
        requestedPeriod >> 10;
        // Set the new sample period in ns.
        samplePeriod = requestedPeriod * 250 * 1024;
    } else
    // If the clock cycle count is greater than (65535 * 64), or divisble by 256.
    if((requestedPeriod > 4194240) || ((requestedPeriod & 255) == 0))
    {
        // This sets the prescalar to 256 and divides the requested cycle count by 256.
        TCCR1B = (1 << CS12) | (1 << WGM12);
        requestedPeriod >> 8;
        // Set the new sample period in ns.
        samplePeriod = requestedPeriod * 250 * 256;
    } else
    // If the clock cycle count is greater than (65535 * 8), or divisble by 64.
    if((requestedPeriod > 524280) || ((requestedPeriod & 63) == 0))
    {
        // This sets the prescalar to 64 and divides the requested cycle count by 64.
        TCCR1B = (1 << CS11) | (1 << CS10) | (1 << WGM12);
        requestedPeriod >> 6;
        // Set the new sample period in ns.
        samplePeriod = requestedPeriod * 250 * 64;
    } else
    // If the clock cycle count is greater than (65535 * 1), or divisble by 8.
    if((requestedPeriod > 65535) || ((requestedPeriod & 7) == 0))
    {
        // This sets the prescalar to 8 and divides the requested cycle count by 8.
        TCCR1B = (1 << CS11) | (1 << WGM12);
        requestedPeriod >> 3;
        // Set the new sample period in ns.
        samplePeriod = requestedPeriod * 250 * 8;
    } else {
        // There are no cycle count constraints and it isn't divisible by 8, so set the prescalar to 1.
        TCCR1B = (1 << CS10) | (1 << WGM12);
        // Set the new sample period in ns.
        samplePeriod = requestedPeriod * 250;
    }

    // Set the number of prescalar adjusted clock cycles TIMER1 should trigger after.
    OCR1A = requestedPeriod;

  #ifdef MDEBUG_MODE
    debugLog(INFO_END_setSamplePeriod);
  #endif
}



inline void getPortStates()
{
  #ifdef MDEBUG_MODE
    debugLog(INFO_START_getPortStates);
  #endif

    // Write the portion of `enabledFlags` storing the states of the Vernier ports into the serial output buffer.
    writeSerialBytes({(uint8_t)(COMMAND_SOURCE_ARDUINO | COMMAND_GET_PORT_STATES),
                      (uint8_t)(enabledFlags & PORT_ALL_ENABLED_BITMASK)});

  #ifdef MDEBUG_MODE
    debugLog(INFO_END_getPortStates);
  #endif
}



inline void setPortStates()
{
  #ifdef MDEBUG_MODE
    debugLog(INFO_START_setPortStates);
  #endif

    // Read the requested port states from the serial input buffer.
    uint8_t requestedPortStates[1];
    readSerialBytes(requestedPortStates);
    // The first 4 bits are a mask of which ports to change the state of (1 indicates the state should be changed).
    uint8_t portStateChangeMask = (requestedPortStates[0] >> 4) & PORT_ALL_ENABLED_BITMASK;

    // Clear the specified port states in `enabledFlags` and write the requested states in their place.
    enabledFlags = (enabledFlags & ~portStateChangeMask) | (requestedPortStates[0] & portStateChangeMask);

  #ifdef MDEBUG_MODE
    debugLog(INFO_END_setPortStates);
  #endif
}



inline void getAnalogPinStates()
{
  #ifdef MDEBUG_MODE
    debugLog(INFO_START_getAnalogPinStates);
  #endif

    // Write the portion of `enabledFlags` storing the states of the analog pins into the serial output buffer.
    writeSerialBytes({(uint8_t)(COMMAND_SOURCE_ARDUINO | COMMAND_GET_APIN_STATES),
                      (uint8_t)(enabledFlags & APIN_ALL_ENABLED_BITMASK)});

  #ifdef MDEBUG_MODE
    debugLog(INFO_END_getAnalogPinStates);
  #endif
}



inline void setAnalogPinStates()
{
  #ifdef MDEBUG_MODE
    debugLog(INFO_START_setAnalogPinStates);
  #endif

    // Read the requested analog pin states from the serial input buffer.
    uint8_t requestedAPinStates[1];
    readSerialBytes(requestedAPinStates);
    // The first 4 bits are a mask of which pins to change the state of (1 indicates the state should be changed).
    uint8_t pinStateChangeMask = requestedAPinStates[0] & APIN_ALL_ENABLED_BITMASK;

    // Clear the specified analog pin states in `enabledFlags` and write the requested states in their place.
    enabledFlags = (enabledFlags & ~pinStateChangeMask) | ((requestedAPinStates[0] << 4) & pinStateChangeMask);

  #ifdef MDEBUG_MODE
    debugLog(INFO_END_setAnalogPinStates);
  #endif
}



inline void getDigitalPinModes()
{
  #ifdef MDEBUG_MODE
    debugLog(INFO_START_getDigitalPinModes);
  #endif

    // Write the `pinModeFlags` bitarray into the serial output buffer.
    writeSerialBytes({(uint8_t)(COMMAND_SOURCE_ARDUINO | COMMAND_GET_DPIN_MODES),
                               pinModeFlags});

  #ifdef MDEBUG_MODE
    debugLog(INFO_END_getDigitalPinModes);
  #endif
}



inline void setDigitalPinModes()
{
  #ifdef MDEBUG_MODE
    debugLog(INFO_START_setDigitalPinModes);
  #endif

    // Read the change mask and requested digital pin states from the serial input buffer.
    // The first byte is a mask of which pins to change the state of (1 indicates the state should be changed).
    // The second byte contains the modes to set the digital pin to (1 indicates input mode, and 0 output).
    uint8_t data[2];
    readSerialBytes(data);

    // Clear the specified digital pin states in `pinModeFlags` and write the requested modes in their place.
    pinModeFlags = (pinModeFlags & ~data[0]) | (data[1] & data[0]);

  #ifdef MDEBUG_MODE
    debugLog(INFO_END_setDigitalPinModes);
  #endif
}



inline void getDigitalPinOutputs()
{
  #ifdef DEBUG_MODE
    debugLog(INFO_START_getDigitalPinOutputs);
  #endif

    // Write the `outputValues` bitarray into the serial output buffer.
    writeSerialBytes({(uint8_t)(COMMAND_SOURCE_ARDUINO | COMMAND_GET_DPIN_OUTPUTS),
                               outputValues});

  #ifdef DEBUG_MODE
    debugLog(INFO_END_getDigitalPinOutputs);
  #endif
}



inline void setDigitalPinOutputs()
{
  #ifdef DEBUG_MODE
    debugLog(INFO_START_setDigitalPinOutputs);
  #endif

    // Read the change mask and requested output values from the serial input buffer.
    // The first byte is a mask of which pins to set the values for (1 indicates the value should be set).
    // The second byte contains the values to set the digital pins to.
    uint8_t data[2];
    readSerialBytes(data);

    // Clear the specified digital pin values in `outputValues` and write the specified values in their place.
    outputValues = (outputValues & ~data[0]) | (data[1] & data[0]);
    PORTB = (PORTB & B11111100) | ((outputValues & B11000000) >> 6);
    PORTD = (PORTD & B00000011) | ((outputValues & B00111111) << 2);

  #ifdef DEBUG_MODE
    debugLog(INFO_END_setDigitalPinOutputs);
  #endif
}



inline void getSensorIDs()
{
  #ifdef MDEBUG_MODE
    debugLog(INFO_START_getSensorIDs);
  #endif

    // Write the sensor IDs to the serial output buffer in order.
    writeSerialBytes({(uint8_t)(COMMAND_SOURCE_ARDUINO | COMMAND_GET_SENSOR_IDS),
                               sensorIDs[0],
                               sensorIDs[1],
                               sensorIDs[2],
                               sensorIDs[3]});

  #ifdef MDEBUG_MODE
    debugLog(INFO_END_getSensorIDs);
  #endif
}



inline void setSensorID()
{
  #ifdef MDEBUG_MODE
    debugLog(INFO_START_setSensorIDs);
  #endif

    // Read in the address of the sensor to override and the ID to override it with from the serial input buffer.
    uint8_t data[2];
    readSerialBytes(data);

  #ifdef MCHECK_MODE
    // If an invalid port address (valid addresses are 0,1,2,3) was provided, report an error and do nothing.
    if(data[0] > 3)
    {
        debugLogWithStack(ERROR_INVALID_PORT_ADDRESS_setSensorID, {data[0]});
        return;
    }
  #endif

    // Override the specified port's sensorID and reconfigure it's associated analog pins to the sensor's default.
    sensorIDs[data[0]] = data[1];
    configureSensorPins(data[0], data[1]);

  #ifdef MDEBUG_MODE
    debugLog(INFO_END_setSensorIDs);
  #endif
}



inline void takeSingleReading()
{
  #ifdef MDEBUG_MODE
    debugLog(INFO_START_takeSingleReading);
  #endif

  #ifdef MCHECK_MODE
    // If there's already a reading in progress, report an error and do nothing.
    const uint8_t currentReadingType = statusFlags & CURRENT_READING_TYPE_BITMASK;
    if(currentReadingType != READING_TYPE_NONE)
    {
        debugLogWithStack(ERROR_ALREADY_READING_takeSingleReading, {currentReadingType});
        return;
    }
  #endif

    // Set that a single sensor reading is running and immediately start a sensor reading.
    statusFlags |= READING_TYPE_SINGLE;
    startNewSensorReading();

  #ifdef MDEBUG_MODE
    debugLog(INFO_END_takeSingleReading);
  #endif
}



inline void startBatchReading()
{
  #ifdef MDEBUG_MODE
    debugLog(INFO_START_startBatchReading);
  #endif

  #ifdef MCHECK_MODE
    // If there's already a reading in progress, report an error and do nothing.
    const uint8_t readingType = statusFlags & CURRENT_READING_TYPE_BITMASK;
    if(readingType != READING_TYPE_NONE)
    {
        debugLogWithStack(ERROR_ALREADY_READING_startBatchReading, {readingType});
        return;
    }
  #endif

    // Set that a batch reading is running and start TIMER1 to schedule and automatically start sensor readings.
    statusFlags |= READING_TYPE_BATCH_WAITING;
    // Restart TIMER1 at 0.
    TCNT1 = 0;
    // Enable the TIMER1 interrupt routine that runs when TIMER1 matches the OCR1A register.
    TIMSK1 = (1 << OCIE1A);

  #ifdef MDEBUG_MODE
    debugLog(INFO_END_startBatchReading);
  #endif
}



inline void startPollReading()
{
  #ifdef MDEBUG_MODE
    debugLog(INFO_START_startPollReading);
  #endif

    // Read the reference values to compare the digital pins against while polling.
    uint8_t referenceValues[1];
    readSerialBytes(referenceValues);

  #ifdef MCHECK_MODE
    // If there's already a reading in progress, report an error and do nothing.
    const uint8_t readingType = statusFlags & CURRENT_READING_TYPE_BITMASK;
    if(readingType != READING_TYPE_NONE)
    {
        debugLogWithStack(ERROR_ALREADY_READING_startPollReading, {readingType});
        return;
    }
  #endif

    // Store the specified reference values, and set that a poll reading is running.
    pollReferenceValues = referenceValues[0];
    statusFlags |= READING_TYPE_POLLING;

  #ifdef MDEBUG_MODE
    debugLog(INFO_END_startPollReading);
  #endif
}



    inline void scanSensors()
    {
      #ifdef MDEBUG_MODE
        debugLog(INFO_START_scanSensors);
      #endif

      #ifdef MCHECK_MODE
        // If there's already a reading in progress, report an error and do nothing.
        const uint8_t readingType = statusFlags & CURRENT_READING_TYPE_BITMASK;
        if(readingType != READING_TYPE_NONE)
        {
            debugLogWithStack(ERROR_ALREADY_READING_scanSensors, {readingType});
            return;
        }
      #endif

        // Set that a sensor ID scan is running.
        statusFlags |= READING_TYPE_IDENTIFY;

        // Start by reading the sensor connected to analog port 1 on the Vernier interface.
        // Set the Vernier interface's MUX address to 0 by clearing it (0 represents analog port 1).
        PORTB &= ~MUX_ADDRESS_BITMASK;
        // Start a reading on analog pin A5 (it's shared between all ports and used to read ID voltages from sensors).
        startAnalogReading(ADMUX_PIN_ADDRESS_A5);

      #ifdef MDEBUG_MODE
        debugLog(INFO_END_scanSensors);
      #endif
    }



inline void stopReading()
{
  #ifdef MDEBUG_MODE
    debugLog(INFO_START_stopReading);
  #endif

    // Switch on the type of reading currently in progress.
    const uint8_t readingType = statusFlags & CURRENT_READING_TYPE_BITMASK;
    switch(readingType)
    {
        case(READING_TYPE_BATCH_WAITING): case(READING_TYPE_BATCH_RUNNING):
            // Stop TIMER1 and it's interrupts.
            TIMSK1 = B00000000;
            // Note that there is no `break` here; this continues to the next case and sets the stop-reading flag.

        case(READING_TYPE_SINGLE): case(READING_TYPE_IDENTIFY):
            // Set a flag indicating the reading should stop itself when next possible.
            statusFlags |= STOP_CURRENT_READING_BITMASK;
        break;

        case(READING_TYPE_POLLING):
            // Set the reading type to none. No additional steps are necessary to stop polling mode.
            // Clearing the current reading type is the same as setting it to `READING_TYPE_NONE`.
            statusFlags &= ~CURRENT_READING_TYPE_BITMASK;
        break;

      #ifdef MCHECK_MODE
        case(READING_TYPE_NONE):
            // If there's no reading to stop, report an error.
            debugLog(ERROR_NO_READING_stopReading);
        break;

        default:
            debugDumpWithStack(ERROR_ILLEGAL_READING_stopReading, {readingType});
        break;
      #endif
    }

  #ifdef MDEBUG_MODE
    debugLog(INFO_END_stopReading);
  #endif
}



inline void configureSensorPins(const uint8_t address, const uint8_t sensorID)
{
  #ifdef MDEBUG_MODE
    debugLogWithStack(INFO_START_configureSensorPins, {address, sensorID});
  #endif

  // TODO

  #ifdef MDEBUG_MODE
    debugLog(INFO_END_configureSensorPins);
  #endif
}



inline void checkTIMER1Interrupt()
{
    // If an interrupt was skipped and left unhandled, report an error, and ignore it. There's nothing we can do.
    if(statusFlags & TIMER_INTERRUPT_MISSED_BITMASK)
    {
        debugLog(ERROR_INTERRUPT_SKIPPED_checkTIMER1Interrupt);
        // Clear the missed-interrupt flag.
        statusFlags &= ~TIMER_INTERRUPT_MISSED_BITMASK;
      #ifdef MDEBUG_MODE
        debugLog(INFO_END_checkTIMER1Interrupt);
      #endif
        return;
    }

    const uint8_t readingType = statusFlags & CURRENT_READING_TYPE_BITMASK;
  #ifdef MCHECK_MODE
    // If the Arduino is supposed to be taking another kind of reading, report an error.
    if((readingType != READING_TYPE_BATCH_WAITING) && (readingType != READING_TYPE_BATCH_RUNNING))
    {
        debugDumpWithStack(ERROR_INVALID_READING_checkTIMER1Interrupt, {readingType});
    }
  #endif

    // If there's a TIMER1 interrupt to handle, handle it.
    if(statusFlags & TIMER_INTERRUPT_SIGNAL_BITMASK)
    {
      #ifdef MDEBUG_MODE
        debugLog(INFO_START_checkTIMER1Interrupt);
      #endif

        // If the last reading is still in progress, report an error, and skip the reading this was supposed to
        // start. Otherwise start a new sensor reading like normal.
        if(readingType == READING_TYPE_BATCH_RUNNING)
        {
            debugLog(ERROR_ALREADY_READING_checkTIMER1Interrupt);
        } else {
            // Change the batch reading type from WAITING to RUNNING, and start a new reading.
            // Setting the new reading type without clearing the old only works here as a micro-optimization.
            // Yeah, yeah. You call it 'the root of all evil', I call it efficient code, whatever.
            statusFlags |= READING_TYPE_BATCH_RUNNING;
            startNewSensorReading();
        }
        // Clear the TIMER1 interrupt flag now that it's been handled.
        statusFlags &= ~TIMER_INTERRUPT_SIGNAL_BITMASK;

      #ifdef MDEBUG_MODE
        debugLog(INFO_END_checkTIMER1Interrupt);
      #endif
    }
  #ifdef MDEBUG_MODE
    else {
        debugLog(INFO_NONE_checkTIMER1Interrupt);
    }
  #endif
}



inline void checkADCInterrupt()
{
  #ifdef MCHECK_MODE
    // If an interrupt was skipped and left unhandled, report an error. This shouldn't ever happen however, since
    // functions check that there's nothing currently running before running a new reading, and the only other
    // place we start analog readings is after handling the last analog reading in this function.
    if(statusFlags & ADC_INTERRUPT_MISSED_BITMASK)
    {
        debugDump(ERROR_INTERRUPT_SKIPPED_checkADCInterrupt);
        // Clear the missed-interrupt flag.
        statusFlags &= ~ADC_INTERRUPT_MISSED_BITMASK;
    }
  #endif

    // If there's an ADC interrupt to handle, handle it.
    if(statusFlags & ADC_INTERRUPT_SIGNAL_BITMASK)
    {
      #ifdef MDEBUG_MODE
        debugLog(INFO_START_checkADCInterrupt);
      #endif

        // Switch on the current type of reading the Arduino is taking.
        const uint8_t readingType = statusFlags & CURRENT_READING_TYPE_BITMASK;
        switch(readingType)
        {
            case(READING_TYPE_SINGLE): case(READING_TYPE_BATCH_RUNNING):
                handleAnalogSensorValueReading();
            break;

            case(READING_TYPE_IDENTIFY):
                handleAnalogSensorIDReading();
            break;

          #ifdef MCHECK_MODE
            case(READING_TYPE_NONE):
                debugDump(ERROR_NO_READING_checkADCInterrupt);
            break;

            case(READING_TYPE_BATCH_WAITING):
                debugDump(ERROR_INVALID_BATCH_TYPE_checkADCInterrupt);
            break;

            default:
                debugDumpWithStack(ERROR_INVALID_READING_checkADCInterrupt, {readingType});
            break;
          #endif
        }
        // Clear the ADC interrupt flag now that it's been handled.
        statusFlags &= ~ADC_INTERRUPT_SIGNAL_BITMASK;

      #ifdef MDEBUG_MODE
        debugLog(INFO_END_checkADCInterrupt);
      #endif
    }
  #ifdef MDEBUG_MODE
    else {
        debugLog(INFO_NONE_checkADCInterrupt);
    }
  #endif
}



inline void handleAnalogSensorValueReading()
{
  #ifdef MDEBUG_MODE
    debugLog(INFO_START_handleAnalogSensorValueReading);
  #endif

    // If this reading was canceled, then complete the reading immediately and return.
    if(statusFlags & STOP_CURRENT_READING_BITMASK)
    {
        completeSensorReading();
      #ifdef MDEBUG_MODE
        debugLog(INFO_END_handleAnalogSensorValueReading);
      #endif
        return;
    }

    const uint8_t address = ADMUX & ADMUX_ADDRESS_BITMASK;
  #ifdef MCHECK_MODE
    // If the reading isn't being taken on a pin connected to the sensors (A0,A1,A2,A3), report an error.
    if(address > ADMUX_PIN_ADDRESS_A3)
    {
        debugDumpWithStack(ERROR_INVALID_ADDRESS_handleAnalogSensorValueReading, {address});
    }
    // If the reading was taken on a disabled pin, report an error.
    if(!(enabledFlags & (1 << (address + 4))))
    {
        debugDumpWithStack(ERROR_DISABLED_ADDRESS_handleAnalogSensorValueReading, {address});
    }
    // If the Arduino isn't currently taking a sensor reading, report an error.
    const uint8_t readingType = statusFlags & CURRENT_READING_TYPE_BITMASK;
    if((readingType != READING_TYPE_SINGLE) && (readingType != READING_TYPE_BATCH_RUNNING))
    {
        debugDumpWithStack(ERROR_INVALID_READING_handleAnalogSensorValueReading, {readingType});
    }
  #endif

  // Select the appropiate buffer and position variable to use to store the sensor reading in.
  #ifndef MAVERAGE_COUNT
    uint8_t* buffer = dataBuffer;
    uint16_t& pos = dataBufferCounter;
  #else
    uint8_t* buffer = averagingBuffer;
    uint16_t& pos = averagingBufferCounter;
  #endif

    const uint8_t memoryMode = (enabledFlags & APIN_ALL_ENABLED_BITMASK) | address;
    // Store the analog reading in the buffer and start the next reading if applicable.
    switch(memoryMode)
    {
        // This is reading 1 out of 1; Store the reading in the buffer and then complete the reading.
        case(B00010000): case(B00100001): case(B01000010): case(B10000011):
            buffer[pos]   = ADCL;
            buffer[pos+1] = (ADCH & B00000011);
            pos += 2;
            completeSensorReading();
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
            pos += 3;
            completeSensorReading();
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
            pos += 4;
            completeSensorReading();
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
            pos += 5;
            completeSensorReading();
        break;

      #ifdef MCHECK_MODE
        // These cases cover the possibility that an interrupt was received for a disabled pin.
        case(B00000000): case(B00000001): case(B00000010): case(B00000011): // No analog pins are enabled.
                          case(B00010001): case(B00010010): case(B00010011): // Analog pin 0 is enabled.
        case(B00100000):                  case(B00100010): case(B00100011): // Analog pin 1 is enabled.
        case(B01000000): case(B01000001):                  case(B01000011): // Analog pin 2 is enabled.
        case(B10000000): case(B10000001): case(B10000010):                  // Analog pin 3 is enabled.
                                          case(B00110010): case(B00110011): // Analog pins 1,2 are enabled.
                          case(B01010001):                  case(B01010011): // Analog pins 1,3 are enabled.
                          case(B10010001): case(B10010010):                  // Analog pins 1,4 are enabled.
        case(B01100000):                                   case(B01100011): // Analog pins 2,3 are enabled.
        case(B10100000):                  case(B10100010):                  // Analog pins 2,4 are enabled.
        case(B11000000): case(B11000001):                                   // Analog pins 3,4 are enabled.
                                                            case(B01110011): // Analog pins 1,2,3 are enabled.
                                          case(B10110010):                  // Analog pins 1,2,4 are enabled.
                          case(B11010001):                                   // Analog pins 1,3,4 are enabled.
        case(B11100000):                                                    // Analog pins 2,3,4 are enabled.
            debugDumpWithStack(ERROR_INVALID_MEMORY_MODE_handleAnalogSensorValueReading, {memoryMode});
        break;

        // None of the cases matched, so an impossible value was switched on.
        default:
            debugDumpWithStack(ERROR_ILLEGAL_MEMORY_MODE_handleAnalogSensorValueReading, {memoryMode});
        break;
      #endif
    }

  #ifdef MDEBUG_MODE
    debugLog(INFO_END_handleAnalogSensorValueReading);
  #endif
}



inline void handleAnalogSensorIDReading()
{
  #ifdef MDEBUG_MODE
    debugLog(INFO_START_handleAnalogSensorIDReading);
  #endif

    // If this reading was canceled, then cancel the sensor scan and return immediately.
    if(statusFlags & STOP_CURRENT_READING_BITMASK)
    {
        // Stop any analog readings, and reset the current reading type to NONE.
        stopAnalogReadings();
      #ifdef MDEBUG_MODE
        debugLog(INFO_END_handleAnalogSensorValueReading);
      #endif
        return;
    }

    const uint8_t portAddress = PORTB & MUX_ADDRESS_BITMASK;

  #ifdef MCHECK_MODE
    // If the port address is invalid, report an error and stop the sensor scan.
    if((portAddress != MUX_PORT_ADDRESS_ANALOG_1) && (portAddress != MUX_PORT_ADDRESS_DIGITAL_1) &&
       (portAddress != MUX_PORT_ADDRESS_ANALOG_2) && (portAddress != MUX_PORT_ADDRESS_DIGITAL_2))
    {
        debugDumpWithStack(ERROR_ILLEGAL_PORT_ADDRESS_handleAnalogSensorIDReading, {portAddress});
        // Clearing the current reading type is the same as setting it to `READING_TYPE_NONE`.
        statusFlags &= ~CURRENT_READING_TYPE_BITMASK;
        stopAnalogReadings();
      #ifdef MDEBUG_MODE
        debugLog(INFO_END_handleAnalogSensorValueReading);
      #endif
        return;
    }

    // If the reading wasn't taken from pin A5, report an error. This is the only pin used for sensor ID readings.
    const uint8_t address = ADMUX & ADMUX_ADDRESS_BITMASK;
    if(address != ADMUX_PIN_ADDRESS_A5)
    {
        debugDumpWithStack(ERROR_INVALID_ADDRESS_handleAnalogSensorIDReading, {address});
    }
  #endif

    // Get the voltage read from the ADC. Every sensor has a unique resistance, and hence voltage, so we can use
    // this value to determine what type of sensor is connected.
    const uint16_t idVoltage = (uint16_t)ADCL | (uint16_t)(ADCH & B00000011);
    const uint8_t sensorID = getSensorID(idVoltage);

    // Update the sensorID if it changed.
    if(sensorIDs[portAddress] != sensorID)
    {
        sensorIDs[portAddress] = sensorID;
        // Notify the client of the new sensorID and the port it's connected to.
        writeSerialBytes({(uint8_t)(COMMAND_SOURCE_ARDUINO | COMMAND_SET_SENSOR_ID),
                                   portAddress,
                                   sensorID});
    }

    // Switch on which port was just identified to know which port to check next, if any.
    switch(portAddress)
    {
        case(MUX_PORT_ADDRESS_ANALOG_1):
            // Connect analog port 2 to the Vernier interface's MUX so it can be checked next and start a reading.
            PORTB = (PORTB & ~MUX_ADDRESS_BITMASK) | MUX_PORT_ADDRESS_ANALOG_2;
            startAnalogReading(ADMUX_PIN_ADDRESS_A5);
        break;

        case(MUX_PORT_ADDRESS_ANALOG_2):
            // Connect digital port 1 to the Vernier interface's MUX so it can be checked next and start a reading.
            PORTB = (PORTB & ~MUX_ADDRESS_BITMASK) | MUX_PORT_ADDRESS_DIGITAL_1;
            startAnalogReading(ADMUX_PIN_ADDRESS_A5);
        break;

        case(MUX_PORT_ADDRESS_DIGITAL_1):
            // Connect digital port 2 to the Vernier interface's MUX so it can be checked next and start a reading.
            PORTB = (PORTB & ~MUX_ADDRESS_BITMASK) | MUX_PORT_ADDRESS_DIGITAL_2;
            startAnalogReading(ADMUX_PIN_ADDRESS_A5);
        break;

        case(MUX_PORT_ADDRESS_DIGITAL_2):
            // There's no more ports to check, so set the current reading type to none and disable readings.
            statusFlags &= ~CURRENT_READING_TYPE_BITMASK;
            stopAnalogReadings();
        break;

      #ifdef MCHECK_MODE
        default:
          // An illegal port was switched on, report an error.
          debugDumpWithStack(ERROR_ILLEGAL_PORT_ADDRESS2_handleAnalogSensorIDReading, {portAddress});
        break;
      #endif
    }

  #ifdef MDEBUG_MODE
    debugLog(INFO_END_handleAnalogSensorIDReading);
  #endif
}



inline void startNewSensorReading()
{
  #ifdef MDEBUG_MODE
    debugLog(INFO_START_startNewSensorReading);
  #endif

  #ifdef MCHECK_MODE
    // If the Arduino isn't currently taking a single or batch reading, report an error, and skip the reading.
    const uint8_t readingType = statusFlags & CURRENT_READING_TYPE_BITMASK;
    if((readingType != READING_TYPE_SINGLE) && (readingType != READING_TYPE_BATCH_RUNNING))
    {
        debugDumpWithStack(ERROR_INVALID_READING_startNewSensorReading, {readingType});
        return;
    }

    // If the data buffer isn't at position 0, report an error; There's still data left in it to write.
    if(dataBufferCounter != 0)
    {
        debugDumpWithStack(ERROR_UNHANDLED_DATA_startNewSensorReading, {
                               (uint8_t)((dataBufferCounter >> 8) & 0xff),
                               (uint8_t) (dataBufferCounter       & 0xff)});
    }
  #endif

  // Select the appropiate buffer and position variables to store the sensor readings in.
  #ifndef MAVERAGE_COUNT
    uint8_t* buffer = dataBuffer;
    const uint8_t pos = dataBufferCounter + 4; //The first 4 bytes will contain the timestamp, and so are skipped.
  #else
    uint8_t* buffer = averagingBuffer;
    const uint8_t pos = averagingBufferCounter;
  #endif

    // Clear the next 6 bytes in the buffer (the maximum amount of space a reading can take). We always clear the
    // max to protect against corruption in the case the client changes pin states in the middle of a reading.
    buffer[pos] = 0; buffer[pos+1] = 0; buffer[pos+2] = 0; buffer[pos+3] = 0; buffer[pos+4] = 0; buffer[pos+5] = 0;

    // Store the time that the reading was started at.
    const uint32_t time = micros();

    // Record the values of any digital pins that are enabled and in input mode.
    if(enabledFlags & PORT_DIGITAL_1_ENABLED_BITMASK)
    {
        // Store the values of pins 2,3,4,5.
        buffer[pos] = ((PORTD & B00111100) >> 2) & pinModeFlags;
    }
    if(enabledFlags & PORT_DIGITAL_2_ENABLED_BITMASK)
    {
        // Store the values of pins 6,7,8,9.
        buffer[pos] |= ((PORTD & B11000000) >> 2) | ((PORTB & B00000011) << 6) & pinModeFlags;
    }

    // Write the start time into the data buffer and increment the respective buffer counters.
    dataBuffer[dataBufferCounter]   = (time >> 24) & 0xff;
    dataBuffer[dataBufferCounter+1] = (time >> 16) & 0xff;
    dataBuffer[dataBufferCounter+2] = (time >> 8)  & 0xff;
    dataBuffer[dataBufferCounter+3] = time         & 0xff;
  #ifndef MAVERAGE_COUNT
    // Increment the data buffer counter by 5 for the timestamp and digital pin data that was just written into it.
    dataBufferCounter += 5;
  #else
    // Increment the data counter by 4 for the timestamp, and the averaging counter by 1 for the digital pin data.
    dataBufferCounter +=4;
    averagingBufferCounter += 1;
  #endif

    // Start a reading on the lowest address analog pin that's enabled (If any of them are enabled).
    if(enabledFlags & PIN_A0_ENABLED_BITMASK)
    {
        startAnalogReading(ADMUX_PIN_ADDRESS_A0);
    } else
    if(enabledFlags & PIN_A1_ENABLED_BITMASK)
    {
        startAnalogReading(ADMUX_PIN_ADDRESS_A1);
    } else
    if(enabledFlags & PIN_A2_ENABLED_BITMASK)
    {
        startAnalogReading(ADMUX_PIN_ADDRESS_A2);
    } else
    if(enabledFlags & PIN_A3_ENABLED_BITMASK)
    {
        startAnalogReading(ADMUX_PIN_ADDRESS_A3);
    } else {
        // Complete the reading if there's no analog pins to take readings from.
        completeSensorReading();
    }

  #ifdef MDEBUG_MODE
    debugLog(INFO_END_startNewSensorReading);
  #endif
}



inline void completeSensorReading()
{
  #ifdef MDEBUG_MODE
    debugLog(INFO_START_completeSensorReading);
  #endif

    const uint8_t readingType = statusFlags & CURRENT_READING_TYPE_BITMASK;
  #ifdef MCHECK_MODE
    // If the Arduino wasn't taking a single or batch sensor reading, report an error.
    if((readingType != READING_TYPE_SINGLE) && (readingType != READING_TYPE_BATCH_RUNNING))
    {
        debugDumpWithStack(ERROR_INVALID_READING_completeSensorReading, {readingType});
    }
  #endif

    // Only process the results if the current reading wasn't canceled.
    if(!(statusFlags & STOP_CURRENT_READING_BITMASK))
    {
      #ifdef MAVERAGE_COUNT
        uint8_t readingCount = (averagingBufferCounter / 6) + 1;
      #ifdef MCHECK_MODE
        // If more than `AVERAGE_COUNT` many readings were taken, report an error, and ignore the extra readings.
        if(readingCount > AVERAGE_COUNT)
        {
            debugDumpWithStack(ERROR_TOO_MANY_READINGS_completeSensorReading, {readingCount});
            readingCount = AVERAGE_COUNT;
        }
      #endif
        // If enough readings have been taken to average together into a real reading, compute the average.
        if(readingCount == AVERAGE_COUNT)
        {
            // Compute the average time by averaging the start and end times of the reading.
            uint32_t time = micros();
            const uint32_t startTime = ((uint32_t)dataBuffer[dataBufferCounter-4] << 24) |
                                       ((uint32_t)dataBuffer[dataBufferCounter-3] << 16) |
                                       ((uint32_t)dataBuffer[dataBufferCounter-2] <<  8) |
                                        (uint32_t)dataBuffer[dataBufferCounter-1];
            // Divides both times by 2 and adds them together, accounting for any rounding errors; The average time.
            time = (time >> 1) + (startTime >> 1) + (B1 & time & startTime);

            // Create a temporary variable for storing the average digital pin values in.
            uint8_t digitalAverages = B00000000;
            if(enabledFlags & PORT_DIGITAL_1_ENABLED_BITMASK)
            {
                // Count the number of times each bit was set.
                uint8_t setCount[4] = {0, 0, 0, 0};
                uint8_t temp;
                for(uint8_t i = 0; i < AVERAGE_COUNT; i++)
                {
                    temp = averagingBuffer[6 * i];
                    if(temp & B00000001){setCount[0]++;}
                    if(temp & B00000010){setCount[1]++;}
                    if(temp & B00000100){setCount[2]++;}
                    if(temp & B00001000){setCount[3]++;}
                }
                // Set all the bits that were set at least 50% (inclusive) of the time in `digitalAverages`.
                if(2 * setCount[0] / AVERAGE_COUNT){digitalAverages  = B00000001;}
                if(2 * setCount[1] / AVERAGE_COUNT){digitalAverages |= B00000010;}
                if(2 * setCount[2] / AVERAGE_COUNT){digitalAverages |= B00000100;}
                if(2 * setCount[3] / AVERAGE_COUNT){digitalAverages |= B00001000;}
            }
            if(enabledFlags & PORT_DIGITAL_2_ENABLED_BITMASK)
            {
                // Count the number of times each bit was set.
                uint8_t setCount[4] = {0, 0, 0, 0};
                uint8_t temp;
                for(uint8_t i = 0; i < AVERAGE_COUNT; i++)
                {
                    temp = averagingBuffer[6 * i];
                    if(temp & B00010000){setCount[0]++;}
                    if(temp & B00100000){setCount[1]++;}
                    if(temp & B01000000){setCount[2]++;}
                    if(temp & B10000000){setCount[3]++;}
                }
                // Set all the bits that were set at least 50% (inclusive) of the time in `digitalAverages`.
                if(2 * setCount[0] / AVERAGE_COUNT){digitalAverages |= B00010000;}
                if(2 * setCount[1] / AVERAGE_COUNT){digitalAverages |= B00100000;}
                if(2 * setCount[2] / AVERAGE_COUNT){digitalAverages |= B01000000;}
                if(2 * setCount[3] / AVERAGE_COUNT){digitalAverages |= B10000000;}
            }
            // Write the digital averages into the data buffer.
            dataBuffer[dataBufferCounter++] = digitalAverages;

            // Compute how many analog pins were used in the reading and switch on the value.
            const uint8_t count = ((statusFlags & PIN_A0_ENABLED_BITMASK)? 1 : 0) +
                                  ((statusFlags & PIN_A1_ENABLED_BITMASK)? 1 : 0) +
                                  ((statusFlags & PIN_A2_ENABLED_BITMASK)? 1 : 0) +
                                  ((statusFlags & PIN_A3_ENABLED_BITMASK)? 1 : 0);
            switch(count)
            {
                case(0):
                    // Do nothing if there aren't any analog readings to average.
                break;

                case(1):
                    uint16_t analogAverages[1] = {0};
                    for(int i = 0; i < AVERAGE_COUNT; i++)
                    {
                        analogAverages[0] +=  ((uint16_t)averagingBuffer[(6*i)+1]) |
                                             (((uint16_t)averagingBuffer[(6*i)+2] & B00000011) << 8);
                    }
                    analogAverages[0] /= AVERAGE_COUNT;
                    dataBuffer[dataBufferCounter]   =  (analogAverages[0] & B11111111);
                    dataBuffer[dataBufferCounter+1] = ((analogAverages[0] & B1100000000) >> 8);
                    dataBufferCounter += 2;
                break;

                case(2):
                    uint16_t analogAverages[2] = {0, 0};
                    for(int i = 0; i < AVERAGE_COUNT; i++)
                    {
                        analogAverages[0] +=  ((uint16_t)averagingBuffer[(6*i)+1]) |
                                             (((uint16_t)averagingBuffer[(6*i)+2] & B00000011) << 8);
                        analogAverages[1] += (((uint16_t)averagingBuffer[(6*i)+2] & B11111100) >> 2) |
                                             (((uint16_t)averagingBuffer[(6*i)+3] & B00001111) << 6);
                    }
                    analogAverages[0] /= AVERAGE_COUNT;
                    analogAverages[1] /= AVERAGE_COUNT;
                    dataBuffer[dataBufferCounter]   =  (analogAverages[0] & B0011111111);
                    dataBuffer[dataBufferCounter+1] = ((analogAverages[0] & B1100000000) >> 8) |
                                                      ((analogAverages[1] & B0000111111) << 2);
                    dataBuffer[dataBufferCounter+2] = ((analogAverages[1] & B1111000000) >> 6);
                    dataBufferCounter += 3;
                break;

                case(3):
                    uint16_t analogAverages[3] = {0, 0, 0};
                    for(int i = 0; i < AVERAGE_COUNT; i++)
                    {
                        analogAverages[0] +=  ((uint16_t)averagingBuffer[(6*i)+1]) |
                                             (((uint16_t)averagingBuffer[(6*i)+2] & B00000011) << 8);
                        analogAverages[1] += (((uint16_t)averagingBuffer[(6*i)+2] & B11111100) >> 2) |
                                             (((uint16_t)averagingBuffer[(6*i)+3] & B00001111) << 6);
                        analogAverages[2] += (((uint16_t)averagingBuffer[(6*i)+3] & B11110000) >> 4) |
                                             (((uint16_t)averagingBuffer[(6*i)+4] & B00111111) << 4);
                    }
                    analogAverages[0] /= AVERAGE_COUNT;
                    analogAverages[1] /= AVERAGE_COUNT;
                    analogAverages[2] /= AVERAGE_COUNT;
                    dataBuffer[dataBufferCounter]   =  (analogAverages[0] & B0011111111);
                    dataBuffer[dataBufferCounter+1] = ((analogAverages[0] & B1100000000) >> 8) |
                                                      ((analogAverages[1] & B0000111111) << 2);
                    dataBuffer[dataBufferCounter+2] = ((analogAverages[1] & B1111000000) >> 6) |
                                                      ((analogAverages[2] & B0000001111) << 4);
                    dataBuffer[dataBufferCounter+3] = ((analogAverages[2] & B1111110000) >> 4);
                    dataBufferCounter += 4;
                break;

                case(4):
                    uint16_t analogAverages[4] = {0, 0, 0, 0};
                    for(int i = 0; i < AVERAGE_COUNT; i++)
                    {
                        analogAverages[0] +=  ((uint16_t)averagingBuffer[(6*i)+1]) |
                                             (((uint16_t)averagingBuffer[(6*i)+2] & B00000011) << 8);
                        analogAverages[1] += (((uint16_t)averagingBuffer[(6*i)+2] & B11111100) >> 2) |
                                             (((uint16_t)averagingBuffer[(6*i)+3] & B00001111) << 6);
                        analogAverages[2] += (((uint16_t)averagingBuffer[(6*i)+3] & B11110000) >> 4) |
                                             (((uint16_t)averagingBuffer[(6*i)+4] & B00111111) << 4);
                        analogAverages[3] += (((uint16_t)averagingBuffer[(6*i)+4] & B11000000) >> 6) |
                                              ((uint16_t)averagingBuffer[(6*i)+5]              << 2);
                    }
                    analogAverages[0] /= AVERAGE_COUNT;
                    analogAverages[1] /= AVERAGE_COUNT;
                    analogAverages[2] /= AVERAGE_COUNT;
                    analogAverages[3] /= AVERAGE_COUNT;
                    dataBuffer[dataBufferCounter]   =  (analogAverages[0] & B0011111111);
                    dataBuffer[dataBufferCounter+1] = ((analogAverages[0] & B1100000000) >> 8) |
                                                      ((analogAverages[1] & B0000111111) << 2);
                    dataBuffer[dataBufferCounter+2] = ((analogAverages[1] & B1111000000) >> 6) |
                                                      ((analogAverages[2] & B0000001111) << 4);
                    dataBuffer[dataBufferCounter+3] = ((analogAverages[2] & B1111110000) >> 4) |
                                                      ((analogAverages[3] & B0000000011) << 6);
                    dataBuffer[dataBufferCounter+4] = ((analogAverages[4] & B1111111100) >> 2);
                    dataBufferCounter += 5;
                break;

              #ifdef MCHECK_MODE
                default:
                  // An illegal number of pins were used in the reading.
                  debugDumpWithStack(ERROR_ILLEGAL_PIN_COUNT_completeSensorReading, {count});
                break;
              #endif
            }
        } else {
            // If there aren't enough readings to average, then shift the counter forward by a reading and return.
            averagingBufferCounter += 6;
          #ifdef MDEBUG_MODE
            debugLog(INFO_END_completeSensorReading);
          #endif
            return;
        }
      #endif

        // Send the data taken during the reading to the client.
        const uint8_t memoryMode = enabledFlags & APIN_ALL_ENABLED_BITMASK;
        switch(memoryMode)
        {
            // If there aren't any analog pins enabled, readings take up 5 bytes.
            case(B00000000):
              #ifdef MCHECK_MODE
                if(dataBufferCounter != 5)
                {
                    debugDumpWithStack(ERROR_BUFFER_COUNTER_MISMATCH_completeSensorReading, {
                                           (uint8_t)((dataBufferCounter >> 8) & 0xff),
                                           (uint8_t) (dataBufferCounter       & 0xff),
                                                      memoryMode});
                    break;
                }
              #endif
                writeSerialBytes({(uint8_t)(COMMAND_SOURCE_ARDUINO | COMMAND_TAKE_SINGLE_READING),
                                           dataBuffer[dataBufferCounter-5],
                                           dataBuffer[dataBufferCounter-4],
                                           dataBuffer[dataBufferCounter-3],
                                           dataBuffer[dataBufferCounter-2],
                                           dataBuffer[dataBufferCounter-1]});
            break;

            // If there's 1 pin enabled, readings take up 7 bytes.
            case(B00010000): case(B00100000): case(B01000000): case(B10000000):
              #ifdef MCHECK_MODE
                if(dataBufferCounter != 7)
                {
                    debugDumpWithStack(ERROR_BUFFER_COUNTER_MISMATCH_completeSensorReading, {
                                           (uint8_t)((dataBufferCounter >> 8) & 0xff),
                                           (uint8_t) (dataBufferCounter       & 0xff),
                                                      memoryMode});
                    break;
                }
              #endif
                writeSerialBytes({(uint8_t)(COMMAND_SOURCE_ARDUINO | COMMAND_TAKE_SINGLE_READING),
                                           dataBuffer[dataBufferCounter-7],
                                           dataBuffer[dataBufferCounter-6],
                                           dataBuffer[dataBufferCounter-5],
                                           dataBuffer[dataBufferCounter-4],
                                           dataBuffer[dataBufferCounter-3],
                                           dataBuffer[dataBufferCounter-2],
                                           dataBuffer[dataBufferCounter-1]});
            break;

            // If there's 2 pins enabled, readings take up 8 bytes.
            case(B00110000): case(B01010000): case(B10010000): case(B01100000): case(B10100000): case(B11000000):
              #ifdef MCHECK_MODE
                if(dataBufferCounter != 8)
                {
                    debugDumpWithStack(ERROR_BUFFER_COUNTER_MISMATCH_completeSensorReading, {
                                           (uint8_t)((dataBufferCounter >> 8) & 0xff),
                                           (uint8_t) (dataBufferCounter       & 0xff),
                                                      memoryMode});
                    break;
                }
              #endif
                writeSerialBytes({(uint8_t)(COMMAND_SOURCE_ARDUINO | COMMAND_TAKE_SINGLE_READING),
                                           dataBuffer[dataBufferCounter-8],
                                           dataBuffer[dataBufferCounter-7],
                                           dataBuffer[dataBufferCounter-6],
                                           dataBuffer[dataBufferCounter-5],
                                           dataBuffer[dataBufferCounter-4],
                                           dataBuffer[dataBufferCounter-3],
                                           dataBuffer[dataBufferCounter-2],
                                           dataBuffer[dataBufferCounter-1]});
            break;

            // If there's 3 pins enabled, readings take up 9 bytes.
            case(B01110000): case(B10110000): case(B11010000): case(B11100000):
              #ifdef MCHECK_MODE
                if(dataBufferCounter != 9)
                {
                    debugDumpWithStack(ERROR_BUFFER_COUNTER_MISMATCH_completeSensorReading, {
                                           (uint8_t)((dataBufferCounter >> 8) & 0xff),
                                           (uint8_t) (dataBufferCounter       & 0xff),
                                                      memoryMode});
                    break;
                }
              #endif
                writeSerialBytes({(uint8_t)(COMMAND_SOURCE_ARDUINO | COMMAND_TAKE_SINGLE_READING),
                                           dataBuffer[dataBufferCounter-9],
                                           dataBuffer[dataBufferCounter-8],
                                           dataBuffer[dataBufferCounter-7],
                                           dataBuffer[dataBufferCounter-6],
                                           dataBuffer[dataBufferCounter-5],
                                           dataBuffer[dataBufferCounter-4],
                                           dataBuffer[dataBufferCounter-3],
                                           dataBuffer[dataBufferCounter-2],
                                           dataBuffer[dataBufferCounter-1]});
            break;

            // If there's 4 pins enabled, readings take up 10 bytes.
            case(B11110000):
              #ifdef MCHECK_MODE
                if(dataBufferCounter != 10)
                {
                    debugDumpWithStack(ERROR_BUFFER_COUNTER_MISMATCH_completeSensorReading, {
                                           (uint8_t)((dataBufferCounter >> 8) & 0xff),
                                           (uint8_t) (dataBufferCounter       & 0xff),
                                                      memoryMode});
                    break;
                }
              #endif
                writeSerialBytes({(uint8_t)(COMMAND_SOURCE_ARDUINO | COMMAND_TAKE_SINGLE_READING),
                                           dataBuffer[dataBufferCounter-10],
                                           dataBuffer[dataBufferCounter-9],
                                           dataBuffer[dataBufferCounter-8],
                                           dataBuffer[dataBufferCounter-7],
                                           dataBuffer[dataBufferCounter-6],
                                           dataBuffer[dataBufferCounter-5],
                                           dataBuffer[dataBufferCounter-4],
                                           dataBuffer[dataBufferCounter-3],
                                           dataBuffer[dataBufferCounter-2],
                                           dataBuffer[dataBufferCounter-1]});
            break;

          #ifdef MCHECK_MODE
            // An illegal memory mode was switched on, report an error.
            default:
                debugDumpWithStack(ERROR_ILLEGAL_MEMORY_MODE_completeSensorReading, {memoryMode});
            break;
          #endif
        }
    } else {
        // If the Arduino was running a batch reading that got canceled, set the current reading type to none.
        if(readingType == READING_TYPE_BATCH_RUNNING)
        {
            // Stop any analog readings, and reset the current reading type to NONE.
            stopAnalogReadings();
        }
        // Clear the stop-reading flag
        statusFlags &= ~STOP_CURRENT_READING_BITMASK;
    }

    // If the Arduino was only taking a single reading, set the reading type to none since it's been taken.
    if(readingType == READING_TYPE_SINGLE)
    {
        // Stop any analog readings, and reset the current reading type to NONE.
        stopAnalogReadings();
    } else
    // If the Arduino is in batch reading mode.
    if(readingType == READING_TYPE_BATCH_RUNNING)
    {
        // Set that the Arduino is still in batch reading mode, but waiting to take the next reading.
        statusFlags = (statusFlags & ~CURRENT_READING_TYPE_BITMASK) | READING_TYPE_BATCH_WAITING;
    }

    // Reset the buffer positions for the next reading.
    dataBufferCounter = 0;
  #ifdef MAVERAGE_COUNT
    averagingBufferCounter = 0;
  #endif

  #ifdef MDEBUG_MODE
    debugLog(INFO_END_completeSensorReading);
  #endif
}



inline void pollDigitalPins()
{
  #ifdef MDEBUG_MODE
    debugLog(INFO_START_pollDigitalPins);
  #endif

  #ifdef MCHECK_MODE
    // If the Arduino isn't currently taking a poll reading, report an error.
    const uint8_t readingType = statusFlags & CURRENT_READING_TYPE_BITMASK;
    if(readingType != READING_TYPE_POLLING)
    {
        debugDumpWithStack(ERROR_INVALID_READING_pollDigitalPins, {readingType});
    }
  #endif

    uint8_t digitalPinStates = B00000000;
    // Read the current states of the enabled digital pins.
    if(enabledFlags & PORT_DIGITAL_1_ENABLED_BITMASK)
    {
        digitalPinStates |= ((PORTD & B00111100) >> 2);
    }
    if(enabledFlags & PORT_DIGITAL_2_ENABLED_BITMASK)
    {
        digitalPinStates |= ((PORTD & B11000000) >> 2) | ((PORTB & B00000011) << 6);
    }
    // Compares all the digital pins against their reference value, but only keep the pins that are in input mode.
    // Every bit in this variable is 1 if the corresponding pin was in input mode and matched, or 0 if it wasn't.
    const uint8_t matches = ~((digitalPinStates ^ pollReferenceValues) & pinModeFlags);

    // If there was at least 1 match, send the matches bitarray to the client.
    if(matches != 0)
    {
        writeSerialBytes({(uint8_t)(COMMAND_SOURCE_ARDUINO | COMMAND_START_POLL_READING),
                                   matches});
    }

  #ifdef MDEBUG_MODE
    debugLog(INFO_END_pollDigitalPins);
  #endif
}



inline void startAnalogReading(const uint8_t address)
{
  #ifdef MDEBUG_MODE
    debugLogWithStack(INFO_START_startAnalogReading, {address});
  #endif

  #ifdef MCHECK_MODE
    // If the address is an unmapped pin, report an error.
    if(address > ADMUX_PIN_ADDRESS_A5)
    {
        debugDumpWithStack(ERROR_ILLEGAL_ADDRESS_startAnalogReading, {address});
    }
    // If the reading is being taken on pin A4, report an error. We never use pin A4.
    if(address == ADMUX_PIN_ADDRESS_A4)
    {
        debugDump(ERROR_PIN_A4_ACCESS_startAnalogReading);
    }
    // If the Arduino isn't currently taking any readings, report an error.
    if((statusFlags & CURRENT_READING_TYPE_BITMASK) == READING_TYPE_NONE)
    {
        debugDump(ERROR_NO_READING_startAnalogReading);
    }
  #endif

    // Directs the ADC use the onboard 5v power rail as it's reference voltage and sets the pin address to measure.
    ADMUX = (1 << REFS0) | address;
    // Enables the ADC, starts a conversion with interrupts enabled, and sets the ADC clock to a multiplier of 16.
    ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADIE) | (1 << ADPS2);

  #ifdef MDEBUG_MODE
    debugLog(INFO_END_startAnalogReading);
  #endif
}



inline void stopAnalogReadings()
{
  #ifdef MDEBUG_MODE
    debugLog(INFO_START_stopAnalogReadings);
  #endif

  #ifdef MCHECK_MODE
      // If there is a reading still running, report an error.
      const uint8_t readingType = statusFlags & CURRENT_READING_TYPE_BITMASK;
      if(readingType != READING_TYPE_NONE)
      {
          debugDumpWithStack(ERROR_READING_STILL_RUNNING_stopAnalogReadings, {readingType});
      }
  #endif

    // Keep the Analog to Digital Converter (ADC) enabled, but disable all other functionality.
    ADCSRA = (1 << ADEN);
    // Clearing the current reading type is the same as setting it to `READING_TYPE_NONE`.
    statusFlags &= ~CURRENT_READING_TYPE_BITMASK;

  #ifdef MDEBUG_MODE
    debugLog(INFO_END_stopAnalogReadings);
  #endif
}



inline uint8_t getSensorID(const uint16_t voltageReading)// TODO THE VALUES IN THIS METHOD SHOULD BE CHECKED OR SOMETHING!
{
  #ifdef MDEBUG_MODE
    debugLogWithStack(INFO_START_getSensorID, {(uint8_t)((voltageReading >> 8) & 0xff),
                                               (uint8_t) (voltageReading       & 0xff)});
  #endif
//TODO make this work
    // Try determining the sensor type with the ID voltage alone.
    if((88 < voltageReading) && (voltageReading < 98)) // 0.43 < idV < 0.48
    {
        return SENSOR_EXERCISE_HEART_RATE_MONITOR;
    } else
    if((127 < voltageReading) && (voltageReading < 139)) // 0.62 < idV < 0.68
    {
        return SENSOR_EKG_SENSOR;
    } else
    if((176 < voltageReading) && (voltageReading < 195)) // 0.86 < idV < 0.95
    {
        return SENSOR_THERMOCOUPLE;
    } else
    if((241 < voltageReading) && (voltageReading < 266)) // 1.18 < idV < 1.30
    {
        // Resistance(4)
        return 127;//TODO
    } else
    if((311 < voltageReading) && (voltageReading < 344)) // 1.52 < idV < 1.68
    {
        return SENSOR_TI_LIGHT_PROBE;
    } else
    if((393 < voltageReading) && (voltageReading < 436)) // 1.92 < idV < 2.13
    {
        return SENSOR_CURRENT_PROBE;
    } else
    if((487 < voltageReading) && (voltageReading < 539)) // 2.38 < idV < 2.63
    {
        return SENSOR_STAINLESS_STEEL_TEMPERATURE_PROBE;
    } else
    if((584 < voltageReading) && (voltageReading < 645)) // 2.85 < idV < 3.15
    {
        return SENSOR_30_VOLT_VOLTAGE_PROBE;
    } else
    if((670 < voltageReading) && (voltageReading < 754)) // 3.27 < idV < 3.68
    {
        return SENSOR_EXTRA_LONG_TEMPERATURE_PROBE;
    } else
    if((762 < voltageReading) && (voltageReading < 791)) // 3.72 < idV < 3.86
    {
        // Voltage +/-10v(2)
        return 127;//TODO
    } else
    if((836 < voltageReading) && (voltageReading < 852)) // 4.08 < idV < 4.16
    {
        // Raw voltage(14)
        return 127;//TODO
    } else
    if((885 < voltageReading) && (voltageReading < 901)) // 4.32 < idV < 4.40
    {
        return SENSOR_CO2_GAS_SENSOR;
    } else
    if((922 < voltageReading) && (voltageReading < 940)) // 4.50 < idV < 4.59
    {
        return SENSOR_O2_GAS_SENSOR;
    } else
    if((950 < voltageReading) && (voltageReading < 969)) // 4.64 < idV < 4.73
    {
        return SENSOR_DIFFERENTIAL_VOLTAGE_PROBE;
    } else
    if((969 < voltageReading) && (voltageReading < 987)) // 4.73 < idV < 4.82
    {
        // Current(9)
        return 127;//TODO
    } else {
        // Try and identify the sensor using I2C Wire communication.
        //TODO
    }

  #ifdef MDEBUG_MODE
    debugLog(INFO_END_getSensorID);
  #endif
}



inline void establishSerialConnection()
{
    // Keep the onboard LED on until the connection is established.
    PORTB |= B00100000;

    // Start a new serial connection and wait until the port is initialized.
    Serial.begin(BAUD_RATE);
    while(!Serial){}

    // Wait until a SERIAL_BROADCAST command is received by the client.
    while(true)
    {
        if(Serial.available() && (Serial.read() == (COMMAND_SOURCE_CLIENT | COMMAND_SERIAL_BROADCAST)))
        {
            break;
        }
    }

    // Respond with a SERIAL_ACCEPT and ensure that it was sent properly.
    if(Serial.write(COMMAND_SOURCE_ARDUINO | COMMAND_SERIAL_ACCEPT) != 1)
    {
        // Continually flash the onboard LED to indicate an error.
        while(true)
        {
            // Toggle the onboard LED once every 256ms
            if(millis() & B10000000)
            {
                PORTB ^= B00100000;
            }
        }
    } else {
        // Flash it once to indicate success.
        const uint32_t startTime = millis();
        PORTB &= ~B00100000;
        while((millis() - startTime) < 1000){}
        PORTB |= B00100000;
    }

    // Wait until a SERIAL_READY command is received from the client, and echo it back to the client.
    while(true)
    {
        if(Serial.available() && (Serial.read() == (COMMAND_SOURCE_CLIENT | COMMAND_SERIAL_READY)))
        {
            break;
        }
    }
}



template <size_t N> inline void readSerialBytes(uint8_t (&buffer)[N])
{
    // Read the specified number of bytes from the serial input stream.
    const uint8_t count = Serial.readBytes(buffer, N);
    if(count != N)
    {
        debugDumpWithStack(ERROR_SERIAL_TIMEOUT_readSerialBytes, {(uint8_t)N, count});
        serialPanicMode();
        return;
    }

    // Compute a checksum for the values read from the serial connection.
    uint8_t computed = 0;
    for(int i = 0; i < N; i++)
    {
        computed ^= buffer[i];
    }

    // Read the client-computed checksum from the serial connection and ensure a match.
    uint8_t checksum[1];
    if(Serial.readBytes(checksum, 1) != 1)
    {
        debugDump(ERROR_MISSING_CHECKSUM_readSerialBytes);
        serialPanicMode();
        return;
    }
    // If there wasn't a match, report an error and enter serial panic mode.
    if(checksum[0] != computed)
    {
        debugDumpWithStack(ERROR_CHECKSUM_MISMATCH_readSerialBytes, {computed, checksum[0]});
        serialPanicMode();
        return;
    }
}



template <size_t N> inline void writeSerialBytes(const uint8_t (&buffer)[N])
{
    // Compute a checksum for the payload data.
    uint8_t computed = 0;
    for(int i = 0; i < N; i++)
    {
        computed ^= buffer[i];
    }

    // Write the payload and checksum byte into the serial output buffer.
    Serial.write(buffer, N);
    Serial.write(computed);
}



inline void serialPanicMode()
{
    // Turn on the on-board LED to indicate an error has occurred.
    PORTB |= B00100000;

    // Cancel any readings currently in progress.
    if((statusFlags & CURRENT_READING_TYPE_BITMASK) != READING_TYPE_NONE)
    {
        stopReading();
    }

    // Send a 'serial panic' error to the client to alert it that the connection needs to be re-established.
    writeSerialBytes({(uint8_t)(COMMAND_SOURCE_ARDUINO | COMMAND_SERIAL_PANIC)});
    // Flush any bytes still in the serial output buffer.
    Serial.flush();

    // Wait 5 seconds before re-establishing the serial connection.
    const uint32_t timeStart = millis();
    while((millis() - timeStart) < 5000)
    {
        // Flash the onboard LED once every 256ms while waiting.
        if(millis() & B10000000)
        {
            // Toggle the onboard LED
            PORTB ^= B00100000;
        }
        // Discard any leftover or incoming serial input.
        while(Serial.available())
        {
            Serial.read();
        }
    }

    // Re-establish the serial connection.
    Serial.end();
    establishSerialConnection();
    // Respond with a SERIAL_READY and ensure that it was sent properly.
    if(Serial.write(COMMAND_SOURCE_ARDUINO | COMMAND_SERIAL_ACCEPT) != 1)
    {
        // Continually flash the onboard LED to indicate an error.
        while(true)
        {
            // Toggle the onboard LED once every 256ms.
            if(millis() & B10000000)
            {
                PORTB ^= B00100000;
            }
        }
    }
    // Turn the onboard LED off now that the serial port is ready.
    PORTB &= ~B00100000;
}



inline void debugLog(const uint8_t debugCode)
{
    // Get the current time to timestamp the log message with.
    const uint32_t time = micros();
    // Write the log message into the serial buffer so it'll be sent to the client.
    const uint8_t message[] = {
        (COMMAND_SOURCE_ARDUINO | COMMAND_DEBUG_LOG),
        debugCode,
        (uint8_t)((time >> 24) & 0xff),
        (uint8_t)((time >> 16) & 0xff),
        (uint8_t)((time >>  8) & 0xff),
        (uint8_t) (time        & 0xff)
    };
    Serial.write(message, 6);
}



template <size_t N> inline void debugLogWithStack(const uint8_t debugCode, const uint8_t (&stack)[N])
{
  #ifdef MCHECK_MODE
    // If the debug-code doesn't have the additional payload flag it shouldn't send a stack, so report an error.
    if(!(debugCode & ADDITIONAL_PAYLOAD_BITMASK))
    {
        debugDumpWithStack(ERROR_DEBUG_CODE_PAYLOAD_MISTMATCH_debugLogWithStack, {debugCode});
    }
  #endif

    debugLog(debugCode);
    Serial.write(stack, N);
}



inline void debugDump(const uint8_t debugCode)
{
  #ifdef MCHECK_MODE
    // If an informational message was passed to this function, report an error. Only error codes should be passed.
    if((debugCode & DEBUG_CODE_TYPE_BITMASK) != DEBUG_CODE_TYPE_ERROR)
    {
        debugDumpWithStack(ERROR_DEBUG_CODE_TYPE_MISTMATCH_debugDump, {debugCode});
    }
  #endif

    // Get the current time to timestamp the log message with.
    const uint32_t time = micros();
    // Write the log message and dump into the serial buffer so it'll be sent to the client.
    const uint8_t message[] = {
        (COMMAND_SOURCE_ARDUINO | COMMAND_DEBUG_LOG),
        debugCode,
        (uint8_t)((time >> 24) & 0xff),
        (uint8_t)((time >> 16) & 0xff),
        (uint8_t)((time >>  8) & 0xff),
        (uint8_t) (time        & 0xff),
        enabledFlags,
        pinModeFlags,
        statusFlags,
        sensorIDs[0],
        sensorIDs[1],
        sensorIDs[2],
        sensorIDs[3],
        (uint8_t)((samplePeriod >> 24) & 0xff),
        (uint8_t)((samplePeriod >> 16) & 0xff),
        (uint8_t)((samplePeriod >>  8) & 0xff),
        (uint8_t) (samplePeriod        & 0xff),
        (uint8_t)((BAUD_RATE >> 24) & 0xff),
        (uint8_t)((BAUD_RATE >> 16) & 0xff),
        (uint8_t)((BAUD_RATE >>  8) & 0xff),
        (uint8_t) (BAUD_RATE        & 0xff),
        (uint8_t)((dataBufferCounter >> 8) & 0xff),
        (uint8_t) (dataBufferCounter       & 0xff),
      #ifdef MAVERAGE_COUNT
        (uint8_t)((averagingBufferCounter >> 8) & 0xff),
        (uint8_t) (averagingBufferCounter       & 0xff),
      #else
        0,
        0,
      #endif
        (uint8_t)((DATA_BUFFER_SIZE >> 8) & 0xff),
        (uint8_t) (DATA_BUFFER_SIZE       & 0xff),
        AVERAGE_COUNT,
        CHECK_MODE,
        DEBUG_MODE,
        PORTB,
        PORTC,
        PORTD,
        DDRB,
        DDRC,
        DDRD,
        DIDR0,
        DIDR1,
        ADMUX,
        ADCSRA,
        ADCSRB,
        ACSR
    };
    Serial.write(message, 42);
}



template <size_t N> inline void debugDumpWithStack(const uint8_t debugCode, const uint8_t (&stack)[N])
{
  #ifdef MCHECK_MODE
    // If the debug-code doesn't have the additional payload flag it shouldn't send a stack, so report an error.
    if(!(debugCode & ADDITIONAL_PAYLOAD_BITMASK))
    {
        debugDumpWithStack(ERROR_DEBUG_CODE_PAYLOAD_MISTMATCH_debugDumpWithStack, {debugCode});
    }
  #endif

    debugDump(debugCode);
    Serial.write(stack, N);
}




//TODO paste the check functions everywhere so they dont have to wait much
//==== LIMITATIONS AND CONCERNS ====//
/**
 Baud rates have to be between 0 and 2^32 (512 MBps)
 Data buffer sizes have to be between 0 and 65536 (64KB)
 Average count has to be between 0 and 256

 Averages on digital sensors are 1 if the sensor was 1 at any point during any of the readings, and only 0 if it was always 0. This isn't a real average.
 The average time is taken between the start time and end time, not the average of every time-point used.

 Digital Sensor 2 takes a little longer to read from than digital sensor 1.
 Analog sensors are read in order one at a time, so higher address analog pins will be read after others if other pins are enabled.

 The average time may not be accurate. It's only the average of the start and the end, not the true average of each reading's timestamp.
 If an even number of readings are averaged together, the average value for digital pins leans towards 1. Ex: If 2 readings are averaged, and a pin's values for those readings was 0, and 1. It's average will be 1.
**/

// TODO SO WHATS THE POINT OF THE DATA BUFFER NOW THEN?
// Sensor ID readings might not work if A5 is pulled up or something or whatever,
