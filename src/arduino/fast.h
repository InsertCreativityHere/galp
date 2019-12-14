
#ifndef FAST_H_INCLUDE
#define FAST_H_INCLUDE

#define MAVERAGE_COUNT 5
#define MCHECK_MODE
#define MDEBUG_MODE

//===== Macros =====//

  // If a baud rate for the serial connection wasn't set in the compiler, use a default rate of 9600bps.
  #ifdef MBAUD_RATE
    // Ensure BAUD_RATE is a positive number.
    #if(MBAUD_RATE <= 0)
      #error BAUD_RATE must be a positive integer!
    #endif
    const uint32_t BAUD_RATE = MBAUD_RATE;
  #else
    const uint32_t BAUD_RATE = 9600;
  #endif

  // If a size for the data buffer wasn't set in the compiler, use a default size of 256 bytes.
  #ifdef MDATA_BUFFER_SIZE
    // Ensure DATA_BUFFER_SIZE is a positive number.
    #if(MDATA_BUFFER_SIZE <= 0)
      #error DATA_BUFFER_SIZE must be a positive integer!
    #endif
    const uint16_t DATA_BUFFER_SIZE = MDATA_BUFFER_SIZE;
  #else
    const uint16_t DATA_BUFFER_SIZE = 256;
  #endif

  // If an average count wasn't set in the compiler, then (by default) averaging is completely disabled.
  #ifdef MAVERAGE_COUNT
    // Ensure AVERAGE_COUNT is a positive number.
    #if(MAVERAGE_COUNT <= 0)
      #error AVERAGE_COUNT must be a positive integer!
    #endif
    const uint8_t AVERAGE_COUNT = MAVERAGE_COUNT;
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
    const uint8_t APIN_ALL_ENABLED_BITMASK = B11110000;

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
    const uint8_t CURRENT_READING_TYPE_BITMASK = B00000111;
        // Indicates the Arduino isn't currently taking a reading.
        const uint8_t READING_TYPE_NONE          = B00000000; //0
        // Indicates the Arduino is currently taking a single sensor value reading (not batch).
        const uint8_t READING_TYPE_SINGLE        = B00000001; //1
        // Both of the following indicate that the Arduino is currently taking a batch sensor reading. However the
        // WAITING version indicates that the Arduino is between readings, and waiting until the end of a sample period
        // to start the next one, while RUNNING indicates the Arduino is in the middle of an actual sensor reading.
        const uint8_t READING_TYPE_BATCH_WAITING = B00000010; //2
        const uint8_t READING_TYPE_BATCH_RUNNING = B00000011; //3
        // Indicates the Arduino is currently in the middle of a polling sensor reading.
        const uint8_t READING_TYPE_POLLING       = B00000100; //4
        // Indicates the Arduino is currently reading the ID voltages of all the Vernier interface's sensor ports.
        const uint8_t READING_TYPE_IDENTIFY      = B00000101; //5
//      Reading types 6 and 7 are unused
    // This bitmask is used to determine if the currently running reading should be stopped. The flag it masks for
    // indicates that the client has signaled for whatever reading is currently running to be ended.
    // To get whether readings should be stopped, use `(READING_IN_PROGRESS_BITMASK & statusFlags)`.
    const uint8_t STOP_CURRENT_READING_BITMASK = B00001000;
    // These bitmasks are used to determine if there was an interrupt that the main loop needs to handle.
    // When the Arduino calls an Interrupt Service Routine (ISR), it sets the corresponding flag with these.
    // To get whether there's an unhandled interrupt use `(X_INTERRUPT_SIGNAL_BITMASK & statusFlags)`.
    // A value of true indicates there's an interrupt to handle, and false indicates there isn't.
        // Indicates there was an interrupt from the Arduino's TIMER1 clock (this only happens during batch readings to
        // signal that a new reading should be started).
        const uint8_t TIMER_INTERRUPT_SIGNAL_BITMASK = B00010000;
        // Indicates there was an interrupt from the Arduino's Analog to Digital Converter (ADC).
        const uint8_t ADC_INTERRUPT_SIGNAL_BITMASK   = B00100000;
    // These bitmasks are used to determine if there was an interrupt that was skipped, and the main loop needs
    // to account for. The client is always notified of skipped interrupts, but for ADC interrupts, often the skipped
    // reading will need to be retaken.
    // To get whether an unhandled interrupt was missed, use `(X_INTERRUPT_MISSED_BITMASK & statusFlags)`.
    // A value of true indicates an unhandled interrupt was skipped, and false indicates there wasn't.
        // Indicates there was an unhandled interrupt from TIMER1 that got skipped because it wasn't handled in time.
        const uint8_t TIMER_INTERRUPT_MISSED_BITMASK = B01000000;
        // Indicates there was an unhandled interrupt from the ADC that got skipped because it wasn't handled in time.
        const uint8_t ADC_INTERRUPT_MISSED_BITMASK   = B10000000;

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
        const uint8_t COMMAND_DEBUG_LOG           = 0;
        const uint8_t COMMAND_GET_SAMPLE_PERIOD   = 1;
        const uint8_t COMMAND_SET_SAMPLE_PERIOD   = 2;
        const uint8_t COMMAND_GET_PORT_STATES     = 3;
        const uint8_t COMMAND_SET_PORT_STATES     = 4;
        const uint8_t COMMAND_GET_APIN_STATES     = 5;
        const uint8_t COMMAND_SET_APIN_STATES     = 6;
        const uint8_t COMMAND_GET_DPIN_MODES      = 7;
        const uint8_t COMMAND_SET_DPIN_MODES      = 8;
        const uint8_t COMMAND_GET_DPIN_OUTPUTS    = 9;
        const uint8_t COMMAND_SET_DPIN_OUTPUTS    = 10;
        const uint8_t COMMAND_GET_SENSOR_IDS      = 11;
        const uint8_t COMMAND_SET_SENSOR_ID       = 12;
        const uint8_t COMMAND_TAKE_SINGLE_READING = 13;
        const uint8_t COMMAND_START_BATCH_READING = 14;
        const uint8_t COMMAND_START_POLL_READING  = 15;
        const uint8_t COMMAND_SCAN_SENSORS        = 16;
        const uint8_t COMMAND_STOP_READING        = 17;
        const uint8_t COMMAND_SERIAL_PANIC        = 28;
        const uint8_t COMMAND_SERIAL_BROADCAST    = 29;
        const uint8_t COMMAND_SERIAL_ACCEPT       = 30;
        const uint8_t COMMAND_SERIAL_READY        = 31;
//      Command codes 18~27 are unused.
    // Bitmask for getting the source of the command. It should be used like `(COMMAND_SOURCE_BITMASK & commandByte)`.
    const uint8_t COMMAND_SOURCE_BITMASK = B11100000;
        // Prefix marker that denotes a packet was sent from the client to the Arduino.
        const uint8_t COMMAND_SOURCE_CLIENT = B01100000;
        // Prefix marker that denotes a packet was sent from the Arduino to the client.
        const uint8_t COMMAND_SOURCE_ARDUINO = B10100000;

    // Bitmask for getting the kind of a sensor (whether it's analog or digital).
    // To get the kind use `(SENSOR_TYPE_BITMASK & sensorID)`.
    const uint8_t SENSOR_TYPE_BITMASK = B10000000;
        // Prefix marker that indicates a sensor is analog.
        const uint8_t SENSOR_TYPE_ANALOG = B00000000;
        // Prefix marker that indicates a sensor is digital.
        const uint8_t SENSOR_TYPE_DIGITAL = B10000000;
    // Sensor IDs: These are used to represent what type of sensor is connected to various ports on the Arduino. Every
    // sensor has a special voltage line with a resistance value that's unique to every type of sensor. The Arduino can
    // read voltages from it to determine which kind of sensor is connected to a specific port (if any).
    // The first bit of every sensor ID is 0 for analog sensors and 1 for digital sensors.
    // A full list of all supported sensors can be found on 'https://www/vernier.com/products/interfaces/labpro/'.
//TODO THESE NEED WORK!
        const uint8_t SENSOR_NONE                                 = 0;
        const uint8_t SENSOR_30_VOLT_VOLTAGE_PROBE                = 1  | SENSOR_TYPE_ANALOG;
        const uint8_t SENSOR_3_AXIS_ACCELEROMETER                 = 2  | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_25_G_ACCELEROMETER                   = 3  | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_ANEMOMETER                           = 4  | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_BAROMETER                            = 5  | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_CALCIUM_ION_SELECTIVE_ELECTRODE      = 6  | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_CBR_2                                = 7  | SENSOR_TYPE_DIGITAL; //Unused
        const uint8_t SENSOR_CONSTANT_CURRENT_SYSTEM              = 8  | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_CHLORIDE_ION_SELECTIVE_ELECTRODE     = 9  | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_CO2_GAS_SENSOR                       = 10 | SENSOR_TYPE_ANALOG;
        const uint8_t SENSOR_COLORIMETER                          = 11 | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_CONDUCTIVITY_PROBE                   = 12 | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_PLATINUM_CELL_CONDUCTIVITY_PROBE     = 13 | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_CHARGE_SENSOR                        = 14 | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_CURRENT_PROBE                        = 15 | SENSOR_TYPE_ANALOG;
        const uint8_t SENSOR_DIGITAL_CONTROL_UNIT                 = 16 | SENSOR_TYPE_DIGITAL; //Unused
        const uint8_t SENSOR_DUAL_RANGE_FORCE_SENSOR              = 17 | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_DISSOLVED_OXYGEN_PROBE               = 18 | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_DIFFERENTIAL_VOLTAGE_PROBE           = 19 | SENSOR_TYPE_ANALOG;
        const uint8_t SENSOR_ELECTRODE_AMPLIFIER                  = 20 | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_EXERCISE_HEART_RATE_MONITOR          = 21 | SENSOR_TYPE_ANALOG;
        const uint8_t SENSOR_EKG_SENSOR                           = 22 | SENSOR_TYPE_ANALOG;
        const uint8_t SENSOR_FLOW_RATE_SENSOR                     = 23 | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_FORCE_PLATE                          = 24 | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_TRIS_COMPATIBLE_FLAT_PH_SENSOR       = 25 | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_GONIOMETER                           = 26 | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_GOMOTION                             = 27 | SENSOR_TYPE_DIGITAL; //Unused
        const uint8_t SENSOR_GAS_PRESSURE_SENSOR                  = 28 | SENSOR_TYPE_ANALOG; //Unused
//      const uint8_t SENSOR_GO_WIRELESS_EXERCISE_HEART_RATE      = 29; //Unused
//      const uint8_t SENSOR_GO_WIRELESS_HEART_RATE               = 30; //Unused
        const uint8_t SENSOR_HIGH_CURRENT_SENSOR                  = 31 | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_HAND_DYNAMOMETER                     = 32 | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_HAND_GRIP_HEART_RATE_MONITOR         = 33 | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_INSTRUMENTATION_AMPLIFIER            = 34 | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_POTASSIUM_ION_SELECTIVE_ELECTRODE    = 35 | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_LOW_G_ACCELEROMETER                  = 36 | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_LIGHT_SENSOR                         = 37 | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_MICROPHONE                           = 38 | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_MOTION_DETECTOR                      = 39 | SENSOR_TYPE_DIGITAL; //Unused
        const uint8_t SENSOR_MAGNETIC_FIELD_SENSOR                = 40 | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_AMMONIUM_ION_SELECTIVE_ELECTRODE     = 41 | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_NITRATE_ION_SELECTIVE_ELECTRODE      = 42 | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_O2_GAS_SENSOR                        = 43 | SENSOR_TYPE_ANALOG;
        const uint8_t SENSOR_VERNIER_OPTICAL_DO_PROBE             = 44 | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_ORP_SENSOR                           = 45 | SENSOR_TYPE_ANALOG; //Unused
//      const uint8_t SENSOR_POWER_AMPLIFIER                      = 46; //Unused
        const uint8_t SENSOR_PAR_SENSOR                           = 47 | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_PH_SENSOR                            = 48 | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_PRESSURE_SENSOR_400                  = 49 | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_PYRANOMETER                          = 50 | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_RELATIVE_HUMIDITY_SENSOR             = 51 | SENSOR_TYPE_ANALOG; //Unused
//      const uint8_t SENSOR_RESPIRATION_MONITOR_BELT             = 52; //Unused
        const uint8_t SENSOR_ROTARY_MOTION_SENSOR                 = 53 | SENSOR_TYPE_DIGITAL; //Unused
        const uint8_t SENSOR_SALINITY_SENSOR                      = 54 | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_SOIL_MOISTURE_SENSOR                 = 55 | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_SPIROMETER                           = 56 | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_SURFACE_TEMPERATURE_SENSOR           = 57 | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_THERMOCOUPLE                         = 58 | SENSOR_TYPE_ANALOG;
        const uint8_t SENSOR_TI_LIGHT_PROBE                       = 59 | SENSOR_TYPE_ANALOG;
        const uint8_t SENSOR_STAINLESS_STEEL_TEMPERATURE_PROBE    = 60 | SENSOR_TYPE_ANALOG;
        const uint8_t SENSOR_TIME_OF_FLIGHT_PAD                   = 61 | SENSOR_TYPE_DIGITAL; //Unused
        const uint8_t SENSOR_EXTRA_LONG_TEMPERATURE_PROBE         = 62 | SENSOR_TYPE_ANALOG;
        const uint8_t SENSOR_TURBIDITY_SENSOR                     = 63 | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_UVA_SENSOR                           = 64 | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_UVB_SENSOR                           = 65 | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_DROP_COUNTER                         = 66 | SENSOR_TYPE_DIGITAL; //Unused
        const uint8_t SENSOR_VERNIER_ENERGY_SENSOR                = 67 | SENSOR_TYPE_ANALOG; //Unused
        const uint8_t SENSOR_VOLTAGE_PROBE                        = 68 | SENSOR_TYPE_ANALOG;
        const uint8_t SENSOR_PHOTOGATE                            = 69 | SENSOR_TYPE_DIGITAL; //Unused
        const uint8_t SENSOR_VERNIER_PROJECTILE_LAUNCHER          = 70 | SENSOR_TYPE_DIGITAL; //Unused
        const uint8_t SENSOR_VERNIER_RADIATION_MONITOR            = 71 | SENSOR_TYPE_DIGITAL; //Unused
//      const uint8_t SENSOR_VERNIER_STRUCTURES_MATERIALS_TESTER  = 72;
        const uint8_t SENSOR_UNKNOWN                              = 127;
//      Sensor IDs 73~126 are unused.

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
    // Debug codes; These are sent to the client to either provide logging information or indicate an error has
    // occurred. These codes are sent as a payload byte after a `DEBUG_LOG` command byte.
    // The first bit of every debug code indicates whether it's informational (0) or represents an error (1), and
    // the second bit is set to 1 if there's additional stack data in the payload. Note this doesn't include the usual
    // payload sent for error codes containing a dump of the Arduino's registers and the program's global variables.
      #ifdef MDEBUG_MODE
        const uint8_t INFO_START_SETUP                          = 0  | DEBUG_CODE_TYPE_ERROR;
        const uint8_t INFO_END_SETUP                            = 1;
        const uint8_t INFO_START_LOOP                           = 2;
        const uint8_t INFO_NONE_processClientCommands           = 3;
        const uint8_t INFO_START_processClientCommands          = 4;
        const uint8_t INFO_END_processClientCommands            = 5;
        const uint8_t INFO_START_getSamplePeriod                = 6;
        const uint8_t INFO_END_getSamplePeriod                  = 7;
        const uint8_t INFO_START_setSamplePeriod                = 8;
        const uint8_t INFO_END_setSamplePeriod                  = 9;
        const uint8_t INFO_START_getPortStates                  = 10;
        const uint8_t INFO_END_getPortStates                    = 11;
        const uint8_t INFO_START_setPortStates                  = 12;
        const uint8_t INFO_END_setPortStates                    = 13;
        const uint8_t INFO_START_getAnalogPinStates             = 14;
        const uint8_t INFO_END_getAnalogPinStates               = 15;
        const uint8_t INFO_START_setAnalogPinStates             = 16;
        const uint8_t INFO_END_setAnalogPinStates               = 17;
        const uint8_t INFO_START_getDigitalPinModes             = 18;
        const uint8_t INFO_END_getDigitalPinModes               = 19;
        const uint8_t INFO_START_setDigitalPinModes             = 20;
        const uint8_t INFO_END_setDigitalPinModes               = 21;
        const uint8_t INFO_START_getDigitalPinOutputs           = 22;
        const uint8_t INFO_END_getDigitalPinOutputs             = 23;
        const uint8_t INFO_START_setDigitalPinOutputs           = 24;
        const uint8_t INFO_END_setDigitalPinOutputs             = 25;
        const uint8_t INFO_START_getSensorIDs                   = 26;
        const uint8_t INFO_END_getSensorIDs                     = 27;
        const uint8_t INFO_START_setSensorIDs                   = 28;
        const uint8_t INFO_END_setSensorIDs                     = 29;
        const uint8_t INFO_START_takeSingleReading              = 30;
        const uint8_t INFO_END_takeSingleReading                = 31;
        const uint8_t INFO_START_startBatchReading              = 32;
        const uint8_t INFO_END_startBatchReading                = 33;
        const uint8_t INFO_START_startPollReading               = 34;
        const uint8_t INFO_END_startPollReading                 = 35;
        const uint8_t INFO_START_stopReading                    = 36;
        const uint8_t INFO_END_stopReading                      = 37;
        const uint8_t INFO_START_scanSensors                    = 38;
        const uint8_t INFO_END_scanSensors                      = 39;
        const uint8_t INFO_START_configureSensorPins            = 40 | ADDITIONAL_PAYLOAD_BITMASK;
        const uint8_t INFO_END_configureSensorPins              = 41;
        const uint8_t INFO_NONE_checkTIMER1Interrupt            = 42;
        const uint8_t INFO_START_checkTIMER1Interrupt           = 43;
        const uint8_t INFO_END_checkTIMER1Interrupt             = 44;
        const uint8_t INFO_NONE_checkADCInterrupt               = 45;
        const uint8_t INFO_START_checkADCInterrupt              = 46;
        const uint8_t INFO_END_checkADCInterrupt                = 47;
        const uint8_t INFO_START_handleAnalogSensorValueReading = 48;
        const uint8_t INFO_END_handleAnalogSensorValueReading   = 49;
        const uint8_t INFO_START_handleAnalogSensorIDReading    = 50;
        const uint8_t INFO_END_handleAnalogSensorIDReading      = 51;
        const uint8_t INFO_START_startNewSensorReading          = 52;
        const uint8_t INFO_END_startNewSensorReading            = 53;
        const uint8_t INFO_START_completeSensorReading          = 54;
        const uint8_t INFO_END_completeSensorReading            = 55;
        const uint8_t INFO_START_pollDigitalPins                = 56;
        const uint8_t INFO_END_pollDigitalPins                  = 57;
        const uint8_t INFO_START_startAnalogReading             = 58 | ADDITIONAL_PAYLOAD_BITMASK;
        const uint8_t INFO_END_startAnalogReading               = 59;
        const uint8_t INFO_START_stopAnalogReadings             = 60;
        const uint8_t INFO_END_stopAnalogReadings               = 61;
        const uint8_t INFO_START_getSensorID                    = 62 | ADDITIONAL_PAYLOAD_BITMASK;
        const uint8_t INFO_END_getSensorID                      = 63;
      #endif
        const uint8_t ERROR_ILLEGAL_READING_TYPE_loop                          = 0  | DEBUG_CODE_TYPE_ERROR;
        const uint8_t ERROR_ILLEGAL_COMMAND_SOURCE_processClientCommands       = 1  | DEBUG_CODE_TYPE_ERROR | ADDITIONAL_PAYLOAD_BITMASK;
        const uint8_t ERROR_DEBUG_LOG_processClientCommands                    = 2  | DEBUG_CODE_TYPE_ERROR;
        const uint8_t ERROR_ILLEGAL_COMMAND_processClientCommands              = 3  | DEBUG_CODE_TYPE_ERROR | ADDITIONAL_PAYLOAD_BITMASK;
        const uint8_t ERROR_UNSTABLE_SAMPLE_PERIOD_setSamplePeriod             = 4  | DEBUG_CODE_TYPE_ERROR | ADDITIONAL_PAYLOAD_BITMASK;
        const uint8_t ERROR_INVALID_PORT_ADDRESS_setSensorID                   = 5  | DEBUG_CODE_TYPE_ERROR | ADDITIONAL_PAYLOAD_BITMASK;
        const uint8_t ERROR_ALREADY_READING_takeSingleReading                  = 6  | DEBUG_CODE_TYPE_ERROR | ADDITIONAL_PAYLOAD_BITMASK;
        const uint8_t ERROR_ALREADY_READING_startBatchReading                  = 7  | DEBUG_CODE_TYPE_ERROR | ADDITIONAL_PAYLOAD_BITMASK;
        const uint8_t ERROR_ALREADY_READING_startPollReading                   = 8  | DEBUG_CODE_TYPE_ERROR | ADDITIONAL_PAYLOAD_BITMASK;
        const uint8_t ERROR_ALREADY_READING_scanSensors                        = 9  | DEBUG_CODE_TYPE_ERROR | ADDITIONAL_PAYLOAD_BITMASK;
        const uint8_t ERROR_NO_READING_stopReading                             = 10 | DEBUG_CODE_TYPE_ERROR;
        const uint8_t ERROR_ILLEGAL_READING_stopReading                        = 11 | DEBUG_CODE_TYPE_ERROR | ADDITIONAL_PAYLOAD_BITMASK;
        const uint8_t ERROR_INTERRUPT_SKIPPED_checkTIMER1Interrupt             = 12 | DEBUG_CODE_TYPE_ERROR;
        const uint8_t ERROR_INVALID_READING_checkTIMER1Interrupt               = 13 | DEBUG_CODE_TYPE_ERROR | ADDITIONAL_PAYLOAD_BITMASK;
        const uint8_t ERROR_ALREADY_READING_checkTIMER1Interrupt               = 14 | DEBUG_CODE_TYPE_ERROR;
        const uint8_t ERROR_INTERRUPT_SKIPPED_checkADCInterrupt                = 15 | DEBUG_CODE_TYPE_ERROR;
        const uint8_t ERROR_NO_READING_checkADCInterrupt                       = 16 | DEBUG_CODE_TYPE_ERROR;
        const uint8_t ERROR_INVALID_BATCH_TYPE_checkADCInterrupt               = 17 | DEBUG_CODE_TYPE_ERROR;
        const uint8_t ERROR_INVALID_READING_checkADCInterrupt                  = 18 | DEBUG_CODE_TYPE_ERROR | ADDITIONAL_PAYLOAD_BITMASK;
        const uint8_t ERROR_INVALID_ADDRESS_handleAnalogSensorValueReading     = 19 | DEBUG_CODE_TYPE_ERROR | ADDITIONAL_PAYLOAD_BITMASK;
        const uint8_t ERROR_DISABLED_ADDRESS_handleAnalogSensorValueReading    = 20 | DEBUG_CODE_TYPE_ERROR | ADDITIONAL_PAYLOAD_BITMASK;
        const uint8_t ERROR_INVALID_READING_handleAnalogSensorValueReading     = 21 | DEBUG_CODE_TYPE_ERROR | ADDITIONAL_PAYLOAD_BITMASK;
        const uint8_t ERROR_INVALID_MEMORY_MODE_handleAnalogSensorValueReading = 22 | DEBUG_CODE_TYPE_ERROR | ADDITIONAL_PAYLOAD_BITMASK;
        const uint8_t ERROR_ILLEGAL_MEMORY_MODE_handleAnalogSensorValueReading = 23 | DEBUG_CODE_TYPE_ERROR | ADDITIONAL_PAYLOAD_BITMASK;
        const uint8_t ERROR_ILLEGAL_PORT_ADDRESS_handleAnalogSensorIDReading   = 24 | DEBUG_CODE_TYPE_ERROR | ADDITIONAL_PAYLOAD_BITMASK;
        const uint8_t ERROR_INVALID_ADDRESS_handleAnalogSensorIDReading        = 25 | DEBUG_CODE_TYPE_ERROR | ADDITIONAL_PAYLOAD_BITMASK;
        const uint8_t ERROR_ILLEGAL_PORT_ADDRESS2_handleAnalogSensorIDReading  = 26 | DEBUG_CODE_TYPE_ERROR | ADDITIONAL_PAYLOAD_BITMASK;
        const uint8_t ERROR_INVALID_READING_startNewSensorReading              = 27 | DEBUG_CODE_TYPE_ERROR | ADDITIONAL_PAYLOAD_BITMASK;
        const uint8_t ERROR_UNHANDLED_DATA_startNewSensorReading               = 28 | DEBUG_CODE_TYPE_ERROR | ADDITIONAL_PAYLOAD_BITMASK;
        const uint8_t ERROR_INVALID_READING_completeSensorReading              = 29 | DEBUG_CODE_TYPE_ERROR | ADDITIONAL_PAYLOAD_BITMASK;
        const uint8_t ERROR_TOO_MANY_READINGS_completeSensorReading            = 30 | DEBUG_CODE_TYPE_ERROR | ADDITIONAL_PAYLOAD_BITMASK;
        const uint8_t ERROR_BUFFER_COUNTER_MISMATCH_completeSensorReading      = 31 | DEBUG_CODE_TYPE_ERROR | ADDITIONAL_PAYLOAD_BITMASK;
        const uint8_t ERROR_ILLEGAL_MEMORY_MODE_completeSensorReading          = 32 | DEBUG_CODE_TYPE_ERROR | ADDITIONAL_PAYLOAD_BITMASK;
        const uint8_t ERROR_INVALID_READING_pollDigitalPins                    = 33 | DEBUG_CODE_TYPE_ERROR | ADDITIONAL_PAYLOAD_BITMASK;
        const uint8_t ERROR_ILLEGAL_ADDRESS_startAnalogReading                 = 34 | DEBUG_CODE_TYPE_ERROR | ADDITIONAL_PAYLOAD_BITMASK;
        const uint8_t ERROR_PIN_A4_ACCESS_startAnalogReading                   = 35 | DEBUG_CODE_TYPE_ERROR | ADDITIONAL_PAYLOAD_BITMASK;
        const uint8_t ERROR_NO_READING_startAnalogReading                      = 36 | DEBUG_CODE_TYPE_ERROR | ADDITIONAL_PAYLOAD_BITMASK;
        const uint8_t ERROR_READING_STILL_RUNNING_stopAnalogReadings           = 37 | DEBUG_CODE_TYPE_ERROR | ADDITIONAL_PAYLOAD_BITMASK;
        const uint8_t ERROR_SERIAL_TIMEOUT_readSerialBytes                     = 38 | DEBUG_CODE_TYPE_ERROR | ADDITIONAL_PAYLOAD_BITMASK;
        const uint8_t ERROR_MISSING_CHECKSUM_readSerialBytes                   = 39 | DEBUG_CODE_TYPE_ERROR | ADDITIONAL_PAYLOAD_BITMASK;
        const uint8_t ERROR_CHECKSUM_MISMATCH_readSerialBytes                  = 40 | DEBUG_CODE_TYPE_ERROR | ADDITIONAL_PAYLOAD_BITMASK;
        const uint8_t ERROR_DEBUG_CODE_PAYLOAD_MISTMATCH_debugLogWithStack     = 41 | DEBUG_CODE_TYPE_ERROR | ADDITIONAL_PAYLOAD_BITMASK;
        const uint8_t ERROR_DEBUG_CODE_TYPE_MISTMATCH_debugDump                = 42 | DEBUG_CODE_TYPE_ERROR | ADDITIONAL_PAYLOAD_BITMASK;
        const uint8_t ERROR_DEBUG_CODE_PAYLOAD_MISTMATCH_debugDumpWithStack    = 43 | DEBUG_CODE_TYPE_ERROR | ADDITIONAL_PAYLOAD_BITMASK;
//      Error debug codes 44~63 are unused.

//===== Global Variables =====//

    // Bitarray that stores which analog pins and ports are enabled and in use on the Aruindo and Vernier interface.
    // 1 indicates the pin/port is enabled and 0 indicates it's disabled. At startup all pins and ports are disabled.
    uint8_t enabledFlags = B00000000;
    // Bitarray that stores the current mode of the digital pins the Vernier Interface uses (2,3,4,5,6,7,8,9).
    // 0 indicates output mode, and 1 indicates input mode. At startup all digital pins are in input mode.
    uint8_t pinModeFlags = B11111111;
    // Bitarray that stores the values being output by the digital pins that the Vernier interface uses
    // (pins 2,3,4,5,6,7,8,9). At startup all the digital pins are set low (0).
    uint8_t outputValues = B00000000;
    // Bitarray that stores various settings and states of the program while it's running.
    // The first 2 bits store the current reading type, bits 3 & 4 are flags for whether there's been an interrupt from
    // the ADC or TIMER1 respectively, bits 5 & 6 are flags indicating if an interrupt was accidentally skipped and
    // bits 6 & 7 are unused currently. At startup none of the flags are set and the reading type is NONE.
    volatile uint8_t statusFlags = B00000000;

    // Bitarray that stores the values to compare digital pins against while polling. The client is notified about any
    // pins that are in input mode and equal their respective value in this array.
    uint8_t pollReferenceValues = B00000000;

    // Stores the period of time the Arduino waits between taking consecutive readings while in batch reading mode.
    // The value is stored in nanoseconds, and at startup this is set to 0.1 seconds.
    uint32_t samplePeriod = 100000000;
    // Stores the IDs of what sensor is connected to each port (0 indicates no sensor is connected).
    // The ID's are stored in the following order: `[ANALOG1, ANALOG2, DIGITAL1, DIGITAL1]`. At startup there are no
    // sensors connected.
    uint8_t sensorIDs[4] = {0, 0, 0, 0};
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
        /** Sends the client the current values being output by digital pins 2,3,4,5,6,7,8,9 in order. **/
        void getDigitalPinOutputs();
        /** Reads 2 bytes from the serial connection and uses them to set the output values of the digital pins. **/
        void setDigitalPinOutputs();
        /** Sends the client the IDs of the sensor connected to each port (0 indicates no sensor is connected). **/
        void getSensorIDs();
        /** Reads 2 bytes from the serial connection and manually overrides a port's connected sensor ID with them. **/
        void setSensorID();
        /** Starts a sensor reading on all the enabled ports. When finished the Arduino sends the data results back to
          * the client. **/
        void takeSingleReading();
        /** Starts a batch reading session with the currently set sample period. One sensor reading is taken every
          * sample period, and when each individual reading is completed, the values are sent to the client. **/
        void startBatchReading();
        /** Starts a poll reading session with the current pin configuration. Digital input pins are continually
          * scanned, and compared against a reference value, and when pins are equal to their reference value, the
          * client is notified with a timestamped message containing all the pins that matched their references. **/
        void startPollReading();
        /** Triggers the Arduino to rescan each port for the sensors connected to them. If any sensors are detected as
          * being removed, changed, or a new sensor was plugged in to any ports, this notifies the client of it. **/
        void scanSensors();
        /** Stops any currently running readings. If the Arduino is in the process of taking a reading, it waits until
          * the current reading has finished, discards the result, and then stops the reading. **/
        void stopReading();

    /** Configures the analog pins attached to a specified port based on the type of sensor connected to it.
      * Different sensors utilize different pins, and this method enables/disables pins to match which the sensor uses.
      * @param address: The port address of the sensor to configure.
      * @param sensorID: The ID of the sensor plugged into the port.**/
    void configureSensorPins(const uint8_t address, const uint8_t sensorID);

    /** Checks if thre's been an interrupt from TIMER1 and if so, starts a new sensor reading. TIMER1 is only
      * running during batch readings, and an interrupt should occur once per sample period at the end. **/
    void checkTIMER1Interrupt();

    /** Checks if there's been an interrupt from the Arduino's ADC and if so handles it and clears the flag.  **/
    void checkADCInterrupt();
        /** Stores the result of a completed analog sensor reading in the data buffer and either starts a reading on
          * the next analog sensor pin (readings are done 1 pin at a time), or completes the reading. **/
        void handleAnalogSensorValueReading();
        /** Determines what sensor is connected to the port the analog reading was taken and notifies the client if the
          * sensor has changed. If there are more ports to identify, it starts an ID reading on the next port,
          * otherwise this calls `stopAnalogReadings`. **/
        void handleAnalogSensorIDReading();

    /** Starts a sensor reading that records the values of every currently enabled port on the Vernier interface into
      * the data buffer. **/
    void startNewSensorReading();
    /** This method gets called whenever a sensor reading is completed. It sends any currently buffered results to the
      * client. For non-batch readings, this also calls `stopAnalogReadings` at the end. **/
    void completeSensorReading();

    /** Checks all digital input pins against their respective polling conditions (whether the pin is high or low).
      * The client is notified of any pins that satisfy their condition with a timestamped message. **/
    void pollDigitalPins();

    /** Starts a new analog reading on the specified analog pin.
      * @param address: The address of the pin to read from.
      *                 This should be one of the `ADMUX_PIN_ADDRESS_X` constants. **/
    void startAnalogReading(const uint8_t address);
    /** Disables the Analog to Digital Converter (ADC) which takes analog readings. **/
    void stopAnalogReadings();

    /** Gets the sensor ID corresponding to the provided ID voltage. Every type of Vernier sensor has a unique range of
      * voltages that can be read from it's ID pin. This voltage can be read from the sensor and used to determine what
      * sensor it is.
      * @param voltageReading: The value read from the sensor's ID line as measured by the ADC (between 0 and 1024).
      * @return: The sensor ID code corresponding to the specified voltage. **/
    uint8_t getSensorID(const uint16_t voltageReading);

    /** Establishes a serial connection with the client. This function blocks until the connection is established and
      * both the client and the Arduino have verified the connection and are ready for communication. After calling
      * this function, the Arduino must later send a 'SERIAL_READY' command to the client. This function does not send
      * 'SERIAL_READY' in case additional setup needs to be performed before the Arduino is can handle client commands
      * and output. This function does wait until 'SERIAL_READY' is received from the client however. **/
    void establishSerialConnection();

    /** Convenience method for reading bytes in from the serial port. If the correct number of bytes can't be read or
      * an error is detected in the serial input, this function calls `serialPanicMode` and waits until a known serial
      * state can be established again.
      * @param buffer: Array to read values in. Values are always placed into the buffer starting at position 0. **/
    template <size_t N>
    void readSerialBytes(uint8_t (&buffer)[N]);
    /** Convenience method for writing bytes to the serial connection. This automatically computes and sends a checksum
      * byte at the end of payload for client side error-detection.
      * @param buffer: Array to write into the serial output buffer. Values are always read starting at index 0. **/
    template <size_t N>
    void writeSerialBytes(const uint8_t (&buffer)[N]);

    /** This function is called whenever an error occured in reading from the serial connection. It notifies the client
      * and attempts to re-establish a new connection from scratch. (The new connection still uses the same underlying
      * serial connection however). This function also flashes the onboard LED 4 times every second while waiting. **/
    void serialPanicMode();

    /** Sends a timestamped log code to the client.
      * @param debugCode: 1 byte code representing the log message to send to the client. **/
    void debugLog(const uint8_t debugCode);
    /** Sends a timestamped log code to the client with an additional payload including the values of variables used
      * within the currently executing function. This is usually any stack-allocated variables, but can be anything.
      * @param debugCode: 1 byte code representing the log message to send to the client.
      * @param stack: An array of variables to send to the client alongside the log message byte. **/
    template <size_t N>
    void debugLogWithStack(const uint8_t debugCode, const uint8_t (&stack)[N]);
    /** Sends a timestamped log code to the client, followed by a dump of the values of every global variable used in
      * the program and some of the Arduino's registers that the program uses.
      * @param debugCode: 1 byte code representing the log message to send to the client. **/
    void debugDump(const uint8_t debugCode);
    /** Sends a timestamped log code to the client, followed by a dump of the values of every global variable used in
      * the program, an array of stack-allocated variable values, and some of the Arduino's registers the program uses.
      * @param debugCode: 1 byte code representing the log message to send to the client.
      * @param stack: An array of variables to send to the client alongside the log message byte and value dump. **/
    template <size_t N>
    void debugDumpWithStack(const uint8_t debugCode, const uint8_t (&stack)[N]);
#endif
