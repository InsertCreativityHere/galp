
#ifndef FAST_H_INCLUDE
#define FAST_H_INCLUDE

//===== Macros =====//

  // If a baud rate for the serial connection wasn't set in the compiler, use a default rate of 9600bps.
  #ifdef MBAUD_RATE
    const uint32_t BAUD_RATE = MBAUD_RATE;
  #else
    const uint32_t BAUD_RATE = 9600;
  #endif

  // If a size for the data buffer wasn't set in the compiler, use a default size of 256 bytes.
  #ifdef MDATA_BUFFER_SIZE
    const uint16_t DATA_BUFFER_SIZE = MDATA_BUFFER_SIZE;
  #else
    const uint16_t DATA_BUFFER_SIZE = 256;
  #endif

  // If an average count wasn't set in the compiler, then (by default) averaging is completely disabled.
  #ifdef MAVERAGE_COUNT
    const uint8_t AVERAGE_COUNT = AVERAGE_COUNT;
  #else
    const uint8_t AVERAGE_COUNT = 0;
  #endif

  // Rigorous checking is disabled by default, unless CHECK_MODE was defined in the compiler.
  #ifdef MCHECK_MODE
    const bool CHECK_MODE = true;
  #else
    const bool CHECK_MODE = false;
  #endif

  // Debugging is disabled by default, unless DEBUG_MODE was defined in the compiler.
  #ifdef MDEBUG_MODE
    const bool DEBUG_MODE = true;
  #else
    const bool DEBUG_MODE = false;
  #endif

//===== Global Constants =====//

    // These bitmasks are used for getting which ports on the Vernier interface are currently enabled.
    // Only ports that are enabled will be measured during sensor readings, and all enabled ports will be read from.
    // A value of true indicates the port is enabled, and a value of false indicates the port is disabled.
    // To get whether a port is enabled or not, use `(PORT_X_ENABLED_BITMASK & enabledFlags)`.
    const uint8_t PORT_ANALOG_1_ENABLED_BITMASK  = B00000001;
    const uint8_t PORT_ANALOG_2_ENABLED_BITMASK  = B00000010;
    const uint8_t PORT_DIGITAL_1_ENABLED_BITMASK = B00000100;
    const uint8_t PORT_DIGITAL_2_ENABLED_BITMASK = B00001000;
    // This bitmask retrieves the entire section of `enabledFlags` that pertains to which ports are enabled or not.
    const uint8_t PORT_ALL_ENABLED_BITMASK = B00001111;
    // These bitmasks are used for getting which analog pins on the Arduino are currently enabled.
    // Only pins that are enabled will be measured during sensor readings, and all enabled ports will be read from.
    // Usuaully which pins are enabled is determined by the type of sensor, different sensors use different pins, and
    // ports with no sensor will have all their pins disabled, but this can all be overriden with SET_APIN_STATES.
    // A value of true indicates the pin is enabled, and a value of false indicates the pin is disabled.
    // To get whether an analog pin is enabled or not use `(PIN_X_ENABLED_BITMASK & enabledFlags)`.
    const uint8_t PIN_A0_ENABLED_BITMASK = B00010000;
    const uint8_t PIN_A1_ENABLED_BITMASK = B00100000;
    const uint8_t PIN_A2_ENABLED_BITMASK = B01000000;
    const uint8_t PIN_A3_ENABLED_BITMASK = B10000000;
    // Bitmask for getting the portion of `enabledFlags` that contains which analog pins are enabled.
    const uint8_t PIN_ALL_ENABLED_BITMASK = B11110000;

    // These bitmasks are used for getting the current pin mode of the digital pins the Vernier interface uses.
    // A value of false indicates the pin is in output mode, and a value of true indicates input mode.
    // To get the pin mode of a digital pin use `(PIN_MODE_DIGITAL_X_BITMASK & pinModeFlags)`.
    const uint8_t PIN_MODE_DIGITAL_2_BITMASK = B00000001;
    const uint8_t PIN_MODE_DIGITAL_3_BITMASK = B00000010;
    const uint8_t PIN_MODE_DIGITAL_4_BITMASK = B00000100;
    const uint8_t PIN_MODE_DIGITAL_5_BITMASK = B00001000;
    const uint8_t PIN_MODE_DIGITAL_6_BITMASK = B00010000;
    const uint8_t PIN_MODE_DIGITAL_7_BITMASK = B00100000;
    const uint8_t PIN_MODE_DIGITAL_8_BITMASK = B01000000;
    const uint8_t PIN_MODE_DIGITAL_9_BITMASK = B10000000;

    // This bitmask is used to get the current type of reading that the Arduino is taking (if any).
    // To get the current reading type use `(CURRENT_READING_TYPE_BITMASK & statusflags)`, and compare the result
    // to one of the `READING_TYPE_X` constants below which represent the different supported reading types.
    const uint8_t CURRENT_READING_TYPE_BITMASK = B00000011;
        // Indicates the Arduino isn't currently taking a reading.
        const uint8_t READING_TYPE_NONE     = B00000000; //0
        // Indicates the Arduino is currently taking a single sensor value reading (not batch).
        const uint8_t READING_TYPE_SINGLE   = B00000001; //1
        // Indicates the Arduino is currently in the middle of a batch sensor value reading.
        const uint8_t READING_TYPE_BATCH    = B00000010; //2
        // Indicates the Arduino is currently reading the ID voltages of all the Vernier interface's sensor ports.
        const uint8_t READING_TYPE_IDENTIFY = B00000011; //3
    // These bitmasks are used to determine if there was an interrupt that the main loop needs to handle.
    // When the Arduino calls an Interrupt Service Routine (ISR), it sets the corresponding flag with these.
    // To get whether there's an unhandled interrupt use `(X_INTERRUPT_SIGNAL_BITMASK & statusFlags)`.
    // A value of true indicates there's an interrupt to handle, and false indicates there isn't.
        // Indicates there was an interrupt from the Arduino's Analog to Digital Converter (ADC).
        const uint8_t ADC_INTERRUPT_SIGNAL_BITMASK   = B00000100;
        // Indicates there was an interrupt from the Arduino's TIMER1 clock (this only happens during batch readings to
        // signal that a new reading should be started).
        const uint8_t TIMER_INTERRUPT_SIGNAL_BITMASK = B00001000;
      #ifdef MCHECK_MODE
        // Indicates there was an unhandled interrupt from the ADC that got skipped because it wasn't handled in time.
        const uint8_t ADC_INTERRUPT_MISSED_BITMASK   = B00010000;
        // Indicates there was an unhandled interrupt from TIMER1 that got skipped because it wasn't handled in time.
        const uint8_t TIMER_INTERRUPT_MISSED_BITMASK = B00100000;
      #endif
//  const uint8_t STATUS_FLAGS_UNUSED_BIT_6 = B01000000;
//  const uint8_t STATUS_FLAGS_UNUSED_BIT_7 = B10000000;

    // This bitmask is used to get the port address currently stored in the Vernier interface's MUltipleXer (MUX).
    // All ports on the interface share analog pins A4 and A5 for resistance and identification voltage readings,
    // in order to measure these on a sensor, the sensor's address must first be set in the MUX. On the Arduino the
    // MUX address is set by using digital pins 10 and 11 (which are controlled with the PORTD register).
    // To get the port address currently stored in the MUX, use `(MUX_ADDRESS_BITMASK & PORTD)`.
    const uint8_t MUX_ADDRESS_BITMASK = B00001100;
        // Addresses for each of the ports on the Vernier interface.
        const uint8_t MUX_PORT_ADDRESS_ANALOG_1  = B00000000; //0
        const uint8_t MUX_PORT_ADDRESS_ANALOG_2  = B00000100; //4
        const uint8_t MUX_PORT_ADDRESS_DIGITAL_1 = B00001000; //8
        const uint8_t MUX_PORT_ADDRESS_DIGITAL_2 = B00001100; //12

    // This bitmask is used to get the pin address stored in the Arduino's Analog to Digital MUltipleXer (ADMUX).
    // Whatever pin has it's address in the ADMUX register is the pin that will be used for analog readings.
    // To get the analog pin address currently stored in the ADMUX, use `(ADMUX_ADDRESS_BITMASK & ADMUX)`.
    const uint8_t ADMUX_ADDRESS_BITMASK = B00001111;
        // Addresses for each of the analog pins on the Arduino.
        const uint8_t ADMUX_PIN_ADDRESS_A0 = B00000000; //0
        const uint8_t ADMUX_PIN_ADDRESS_A1 = B00000001; //1
        const uint8_t ADMUX_PIN_ADDRESS_A2 = B00000010; //2
        const uint8_t ADMUX_PIN_ADDRESS_A3 = B00000011; //3
        const uint8_t ADMUX_PIN_ADDRESS_A4 = B00000100; //4
        const uint8_t ADMUX_PIN_ADDRESS_A5 = B00000101; //5
//      Addresses 6~15 are only available on larger Arduinos.

    // Bitmask for getting the command code. It should be used like `(COMMAND_CODE_BITMASK & commandByte)`.
    const uint8_t COMMAND_CODE_BITMASK = B00001111;
        // Commands bytes; These are sent between the client and the Arduino to relay information and instructions to
        // one another and make up the first byte of every packet. These consist of a source marker that denotes
        // whether the packet was sent from the client or the Arduino, followed by a command code which encodes the
        // actual command. A list of all supported command codes is as follows:
        const uint8_t COMMAND_DEBUG_LOG           = B0000; //0
        const uint8_t COMMAND_GET_SAMPLE_PERIOD   = B0001; //1
        const uint8_t COMMAND_SET_SAMPLE_PERIOD   = B0010; //2
        const uint8_t COMMAND_GET_PORT_STATES     = B0011; //3
        const uint8_t COMMAND_SET_PORT_STATES     = B0100; //4
        const uint8_t COMMAND_GET_APIN_STATES     = B0101; //5
        const uint8_t COMMAND_SET_APIN_STATES     = B0110; //6
        const uint8_t COMMAND_GET_DPIN_MODES      = B0111; //7
        const uint8_t COMMAND_SET_DPIN_MODES      = B1000; //8
        const uint8_t COMMAND_GET_SENSOR_IDS      = B1001; //9
        const uint8_t COMMAND_SET_SENSOR_IDS      = B1010; //10
        const uint8_t COMMAND_TAKE_READING        = B1011; //11
        const uint8_t COMMAND_START_BATCH_READING = B1100; //12
        const uint8_t COMMAND_STOP_BATCH_READING  = B1101; //13
        const uint8_t COMMAND_SCAN_SENSORS        = B1110; //14
        const uint8_t COMMAND_READY               = B1111; //15
    // Bitmask for getting the source of the command. It should be used like `(COMMAND_SOURCE_BITMASK & commandByte)`.
    const uint8_t COMMAND_SOURCE_BITMASK = B11110000;
        // Prefix marker that denotes a packet was sent from the client to the Arduino.
        const uint8_t COMMAND_SOURCE_CLIENT = B00110000;
        // Prefix marker that denotes a packet was sent from the Arduino to the client.
        const uint8_t COMMAND_SOURCE_ARDUINO = B11010000;

    // Debug codes; These are sent to the client to either provide logging information or indicate an error has
    // occured. These codes are sent as the first payload byte after a `DEBUG_LOG` command byte.
    // The first bit of every debug code indicates whether it's informational (0) or represents an error (1), and
    // the second bit is set to 1 if there's additional stack data in the payload. Note this doesn't include the usual
    // payload sent for error codes containing a dump of the Arduino's registers and the programs gloval variables.
      #ifdef MDEBUG_MODE
//      const uint8_t INFO_CODE_RESERVED_0                      = B00000000; //0
        const uint8_t INFO_START_SETUP                          = B00000001; //1
        const uint8_t INFO_END_SETUP                            = B00000010; //2
        const uint8_t INFO_START_LOOP                           = B00000011; //3
        const uint8_t INFO_START_processClientCommands          = B00000100; //4
        const uint8_t INFO_END_processClientCommands            = B00000101; //5
        const uint8_t INFO_START_getSamplePeriod                = B00000110; //6
        const uint8_t INFO_END_getSamplePeriod                  = B00000111; //7
        const uint8_t INFO_START_setSamplePeriod                = B00001000; //8
        const uint8_t INFO_END_setSamplePeriod                  = B00001001; //9
        const uint8_t INFO_START_getPortStates                  = B00001010; //10
        const uint8_t INFO_END_getPortStates                    = B00001011; //11
        const uint8_t INFO_START_setPortStates                  = B00001100; //12
        const uint8_t INFO_END_setPortStates                    = B00001101; //13
        const uint8_t INFO_START_getAnalogPinStates             = B00001110; //14
        const uint8_t INFO_END_getAnalogPinStates               = B00001111; //15
        const uint8_t INFO_START_setAnalogPinStates             = B00010000; //16
        const uint8_t INFO_END_setAnalogPinStates               = B00010001; //17
        const uint8_t INFO_START_getDigitalPinModes             = B00010010; //18
        const uint8_t INFO_END_getDigitalPinModes               = B00010011; //19
        const uint8_t INFO_START_setDigitalPinModes             = B00010100; //20
        const uint8_t INFO_END_setDigitalPinModes               = B00010101; //21
        const uint8_t INFO_START_getSensorIDs                   = B00010110; //22
        const uint8_t INFO_END_getSensorIDs                     = B00010111; //23
        const uint8_t INFO_START_setSensorIDs                   = B00011000; //24
        const uint8_t INFO_END_setSensorIDs                     = B00011001; //25
        const uint8_t INFO_START_takeSingleReading              = B00011010; //26
        const uint8_t INFO_END_takeSingleReading                = B00011011; //27
        const uint8_t INFO_START_startBatchReading              = B00011100; //28
        const uint8_t INFO_END_startBatchReading                = B00011101; //29
        const uint8_t INFO_START_stopBatchReading               = B00011110; //30
        const uint8_t INFO_END_stopBatchReading                 = B00011111; //31
        const uint8_t INFO_START_scanSensors                    = B00100000; //32
        const uint8_t INFO_END_scanSensors                      = B00100001; //33
        const uint8_t INFO_START_checkTIMER1Interrupt           = B00100010; //34
        const uint8_t INFO_END_checkTIMER1Interrupt             = B00100011; //35
        const uint8_t INFO_START_checkADCInterrupt              = B00100100; //36
        const uint8_t INFO_END_checkADCInterrupt                = B00100101; //37
        const uint8_t INFO_START_handleAnalogSensorValueReading = B00100110; //38
        const uint8_t INFO_END_handleAnalogSensorValueReading   = B00100111; //39
        const uint8_t INFO_START_handleAnalogSensorIDReading    = B00101000; //40
        const uint8_t INFO_END_handleAnalogSensorIDReading      = B00101001; //41
        const uint8_t INFO_START_startNewSensorReading          = B00101010; //42
        const uint8_t INFO_END_startNewSensorReading            = B00101011; //43
        const uint8_t INFO_START_completeSensorReading          = B00101100; //44
        const uint8_t INFO_END_completeSensorReading            = B00101101; //45
        const uint8_t INFO_START_startAnalogReading             = B00101110; //46
        const uint8_t INFO_END_startAnalogReading               = B00101111; //47
        const uint8_t INFO_START_stopAnalogReadings             = B00110000; //48
        const uint8_t INFO_END_stopAnalogReadings               = B00110001; //49
//      Informational debug codes 50~63 are unused.
      #endif
      #ifdef MCHECK_MODE
      #endif
    // Bitmask for getting the type of a debug code (informational or error).
    // To get the type of a debug code use `(DEBUG_CODE_TYPE_BITMASK & debugCode)`. And compare with the below values.
    const uint8_t DEBUG_CODE_TYPE_BITMASK = B10000000;
        // Prefix bit that means a debug code represents an informational message.
        const uint8_t DEBUG_CODE_TYPE_INFORM = B00000000;
        // Prefix bit that means a debug code represents an error.
        const uint8_t DEBUG_CODE_TYPE_ERROR  = B10000000;
    // Bitmask for getting whether a debug code has additional payload bytes.
    // To get whether it does, use `(ADDITIONAL_PAYLOAD_BITMASK & debugCode)`. If true there's additional payload data.
    const uint8_t ADDITIONAL_PAYLOAD_BITMASK = B01000000;

//===== Global Variables =====//

    // Bitarray that stores which pins and ports are enabled and in use on the Aruindo and Vernier interface.
    // 1 indicates the pin/port is enabled and 0 indicates it's disabled. At startup all pins and ports are disabled.
    uint8_t enabledFlags = B00000000;
    // Bitarray that stores the current mode of the digital pins the Vernier Interface uses (2,3,4,5,6,7,8,9).
    // 0 indicates output mode, and 1 indicates input mode. At startup all digital pins are in input mode.
    uint8_t pinModeFlags = B11111111;
    // Bitarray that stores various settings and states of the program while it's running.
    // The first 2 bits store the current reading type, bits 3 & 4 are flags for whether there's been an interrupt from
    // the ADC or TIMER1 respectively, bits 5 & 6 are flags indicating if an interrupt was accidentally skipped and
    // bits 6 & 7 are unused currently. At startup none of the flags are set and the reading type is NONE.
    volatile uint8_t statusFlags = B00000000;

    // Stores the period of time the Arduino waits between taking consecutive readings while in batch reading mode.
    // The value is stored in nanoseconds, and at startup this is set to 0.1 seconds.
    uint32_t samplePeriod = 100000000;
    // Stores the IDs of what sensor is connected to each port (0 indicates no sensor is connected).
    // The ID's are stored in the following order: `[ANALOG1, ANALOG2, DIGITAL1, DIGITAL1]`. At startup there are no
    // sensors connected.
    uint8_t sensorIDs[4] = {0, 0, 0, 0}
    // Buffer for temporarily storing reading data before writing it to the client. The size is controller via macro.
    uint8_t dataBuffer[DATA_BUFFER_SIZE];
    // Variable that counts the number of bytes currently stored in the data buffer.
    uint16_t dataBufferCounter = 0;

  #ifdef MAVERAGE_COUNT
    // Buffer for temporarily storing readings to be averaged together before writing the average into the data buffer.
    // A full reading of all sensors takes 6 bytes, so we allocate the maximum possible number of bytes that could be
    // necessary to avoid future re-allocations.
    uint8_t averagingBuffer[AVERAGE_COUNT * 6];
    // Variable that counts the number of readings currently stored in the averaging buffer.
    uint16_t averagingBufferCounter = 0;
  #endif

//===== Functions =====//

    /** Checks for incoming command bytes from the client and if present calls the corresponding command function. **/
    void processClientCommands();
        /** Sends the current sample period to the client. **/
        void getSamplePeriod();
        /** Reads the requested sample period from the serial connection and sets the Arduino's sample period based on
          * it. The Arduino is free to only approximate the provided period, and even ignore it all together (an error
          * will be reported however) if the period is too small for the Arduino to support. Afterwards, this function
          * calls `getSamplePeriod` so the client knows what the actual sample period is.**/
        void setSamplePeriod();
        /** Sends the client the current states of the Vernier interface ports (if they're enabled or disabled). **/
        void getPortStates();
        /** Reads a byte from the serial connection and uses it to manually override the Vernier port states. **/
        void setPortStates();
        /** Sends the client the current states of the analog pins (whether they're enabled or disabled). **/
        void getAnalogPinStates();
        /** Reads a byte from the serial connection and uses it to manually override the analog pin states. **/
        void setAnalogPinStates();
        /** Sends the client the current states of the digital pins (whether they're in input or output mode). **/
        void getDigitalPinModes();
        /** Reads two bytes from the serial connection and uses them to manually override the digital pin modes. **/
        void setDigitalPinModes();
        /** Sends the client the IDs of the sensor connected to each port (0 indicates no sensor is connected). **/
        void getSensorIDs();
        /** Reads 2 bytes from the serial connection and manually overrides a port's connected sensor ID with them. **/
        void setSensorID();
        /** Starts a sensor reading on all the enabled ports. When finished the Arduino sends the data results back to
          * the client. **/
        void takeSingleReading();
        /** Starts a batch reading session with the currently set sample period. One sensor reading is taken every
          * sample period, and when the individual reading is completed, the reading is sent to the client. **/
        void startBatchReading();
        /** Stops any currently running batch sessions. If the Arduino is in the process of taking a reading, it waits
          * until the reading is completed before halting the session. **/
        void stopBatchReading();
        /** Triggers the Arduino to rescan each port for the sensors connected to them. If any sensors are detected as
          * being removed, changed, or a new sensor was plugged in to any ports, this notifies the client of it. **/
        void scanSensors();

    /** Checks if thre's been an interrupt from TIMER1 and if so, starts a new sensor reading. TIMER1 is only
      * running during batch readings, and an interrupt should occur once per sample period at the end. **/
    void checkTIMER1Interrupt();

    /** Checks if there's been an interrupt from the Arduino's ADC and if so handles it and clears the flag.  **/
    void checkADCInterrupt();
        /** Stores the result of a completed analog sensor reading in the data buffer and either starts a reading on
            * the next analog sensor pin (readings are done 1 pin at a time), or completes the reading. **/
        void handleAnalogSensorValueReading(const uint8_t address);
        /** Determines what sensor is connected to the port the analog reading was taken and notifies the client if the
            * sensor has changed. If there are more ports to identity, it starts an ID reading on the next port,
            * otherwise this calls `stopAnalogReadings`. **/
        void handleAnalogSensorIDReading();

    /** Starts a sensor reading that records the values of every currently enabled port on the Vernier interface into
      * the data buffer. **/
    void startNewSensorReading();
    /** This method gets called whenever a sensor reading is completed. It sends any currently buffered results to the
      * client. For none batch readings, this also calls `stopAnalogReadings` at the end. **/
    void completeSensorReading();

    /** Starts a new analog reading on the specified analog pin.
      * @param address: The address of the pin to read from.
      *                 This should be one of the `ADMUX_PIN_ADDRESS_X` constants. **/
    void startAnalogReading(const uint8_t address);
    /** Stops any currently running analog readings and resets the current reading type to `NONE`. **/
    void stopAnalogReadings();

    /** Convenience method for reading bytes in from the serial port. If the correct number of bytes can't be read this
      * function calls `serialPanicMode` and waits until a known serial state can be reached again.
      * @param buffer: Array to read values in. Values are always placed into the buffer starting at position 0.
      * @param length: The number of bytes to try and read from the serial connection. **/
    void readSerialBytes(const uint8_t[] buffer, const uint8_t length);

    /** This function is called whenever an error occured in reading from the serial connection. It notifies the client
      * and attempts to re-establish a new connection from scratch. (The new connection still uses the same underlying
      * serial connection however). This function also flashes the onboard LED 4 times every second while waiting. **/
    void serialPanicMode();

  // Informational log messages are only sent in debug mode.
  #ifdef MDEBUG_MODE
    /** Sends a timestamped log code to the client.
      * @param debugCode: 1 byte code representing the log message to send to the client. **/
    void debugLog(const uint8_t debugCode);
    /** Sends a timestamped log code to the client with an additional payload including the values of variables used
      * within the currently executing function. This is usually any stack-allocated variables, but can be anything.
      * @param debugCode: 1 byte code representing the log message to send to the client.
      * @param stack: An array of variables to send to the client alongside the log message byte.
      * @param stackLength: the number of elements in the stack array. **/
    void debugLogWithStack(const uint8_t debugCode, const uint8_t[] stack, const uint8_t stackLength);
  #endif
    /** Sends a timestamped log code to the client, followed by a dump of the values of every global variable used in
      * the program and some of the Arduino's registers that the program uses.
      * @param debugCode: 1 byte code representing the log message to send to the client. **/
    void debugDump(const uint8_t debugCode);
    /** Sends a timestamped log code to the client, followed by a dump of the values of every global variable used in
      * the program, an array of stack-allocated variable values, and some of the Arduino's registers the program uses.
      * @param debugCode: 1 byte code representing the log message to send to the client.
      * @param stack: An array of variables to send to the client alongside the log message byte and value dump.
      * @param stackLength: the number of elements in the stack array. **/
    void debugDumpWithStack(const uint8_t debugCode, const uint8_t[] stack, const uint8_t stackLength);

#endif
