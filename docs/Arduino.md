
# Arduino Documentation
Galp includes inbuilt support for interfacing with Vernier sensors via an Arduino over a serial connection.
Galp can upload sketches (this is what Arduino programs are called) to Arduinos during run-time, allowing it to use any Arduino, instead of only those pre-loaded with the right sketch.
It includes 2 sketches which both implement the custom protocol galp uses to transmit and receive data and commands;
they can be found in the src/arduino folder. There is `simple.ino` and `fast.ino`, both of which are fully
self-contained stand-alone sketchs.

`simple.ino` uses the normal high-level Arduino API (Application Programming Interface) giving it moderate readability and clarity, as well
as flexibility; there's a good chance that `simple.ino` will run on many different Arduino board models.

`fast.ino` takes the opposite approach and uses direct low-level registery logic to program the Arduino. It
will almost certainly brick any Arduino other than the 'Arduino Uno Rev3'. While this approach provides near-perfect
timing accuracy, and supports much faster sampling frequencies, it's almost completely incomprehensible to anyone
who doesn't have the ATMEGA328D datasheet by their bedside (this is the processor that the Arduino Uno Rev3 uses).

This document exists to explain both the concepts and logic used within `fast.ino` and document the code more
rigorously than what is present within the actual source file. `simple.ino` is deemed to be sufficiently documented
with it's own in-source comments and isn't discussed any further than this paragraph.

Even though this documentation attempts to be rigorous, it doesn't even begin to approach the level of detail and
precision that can be found in the ATMEGA328D datasheet, so for a truly complete and comprehensive understanding check out the [Datasheet](http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf).

Within this document, we first cover the concepts and logic behind the operations and structures used within this program in quick purpose-written tutorials, then afterwards, cover the way they're used within the program, along with other implementation specific details.

--------------------------------

## Macros

## Batch Readings
-mention how sensors are enabled for batch readings

## Memory

## pinModes
-mention pullup resistors

## ADC

## Bitwise Operations

## Registers

## Volatile keyword

## Binary Notation

## Interrupts

## Timers

## Data buffer






# Implementation Details

## Macros
---------
We make frequent use of macros in the code for 2 main purposes:
first we use it to replace complicated operations or variables with more understandable text versions, for instance using `DIGITAL_1_ENABLED` instead of `B01000000`,
and second, to create variables that can be specified by GALP at upload time. This is much simpler than passing variables over the serial connection (especially variables that should be known at the Arduino's startup), and also creates faster code, since the compiler will already know the values for these constants at upload time.

The following list contains every macro used by the program which can be overridden or defined by GALP at upload time:
- `BAUD_RATE`: Sets the baud rate of the serial connection the Arduino uses to communicate with the client. Within this context, this is equivalent to bits per second (Defaults to 9600). For a list of supported baud rates check out [this webpage](https://www.arduino.cc/en/Serial.Begin).
- `SRAM_SIZE`: The storage size of the onboard SRAM in bytes. This value is specific to every board, for the Arduino Uno Rev3 it's 2048 bytes(Defaults to 2048).
- `DEBUG`:     Specifies that the program should run in debug mode, this causes log messages to be printed over the serial connection (By default this value isn't defined, just defining it (even without a value) will enable debug mode).
- `BACKUP`:    Specifies whether the program should back itself up before it starts operating. If this is enabled all of the Arduino's core settings and registers are copied into EEPROM storage and can be reloaded later with digital pin 12 (by pressing the button on the Vernier interface). See [`backupSave`](#backupsave) and [`backupLoad`](#backupload).

The following list contains every macro used internally by the program which can't be set externally:
- `ANALOG_1_CONNECTED` =  (`B00000001`): Bitmask that when ANDed with [`sensorFlags`](#sensorflags) specifies whether the analog1 sensor is connected (0 means not connected, any other value indicates its connected).
- `ANALOG_2_CONNECTED` =  (`B00000010`): Bitmask that when ANDed with [`sensorFlags`](#sensorflags) specifies whether the analog2 sensor is connected (0 means not connected, any other value indicates its connected).
- `DIGITAL_1_CONNECTED` = (`B00000100`): Bitmask that when ANDed with [`sensorFlags`](#sensorflags) specifies whether the digital1 sensor is connected (0 means not connected, any other value indicates its connected).
- `DIGITAL_2_CONNECTED` = (`B00001000`): Bitmask that when ANDed with [`sensorFlags`](#sensorflags) specifies whether the digital2 sensor is connected (0 means not connected, any other value indicates its connected).
- `A0_ENABLED` =          (`B00010000`): Bitmask that when ANDed with [`sensorFlags`](#sensorFlags) specifies whether the A0 pin is used by any of the connected analog sensors (0 means not used, any other value indicates it is used).
- `A1_ENABLED` =          (`B00100000`): Bitmask that when ANDed with [`sensorFlags`](#sensorFlags) specifies whether the A1 pin is used by any of the connected analog sensors (0 means not used, any other value indicates it is used).
- `A2_ENABLED` =          (`B01000000`): Bitmask that when ANDed with [`sensorFlags`](#sensorFlags) specifies whether the A2 pin is used by any of the connected analog sensors (0 means not used, any other value indicates it is used).
- `A3_ENABLED` =          (`B10000000`): Bitmask that when ANDed with [`sensorFlags`](#sensorFlags) specifies whether the A3 pin is used by any of the connected analog sensors (0 means not used, any other value indicates it is used).
- `READING_TYPE` =        (`B00000011`): Bitmask that when ANDed with [`statusFlags`](#statusflags) specifies what type the most recent type of reading was (0 indicates a sensor ID scan, 1 indicates a non-batch sensor reasing, 2 indicates a batch reading, 3 indicates a batch reading and that a reading has just finished (and hasn't been handled)).
- `READING_IN_PROGRESS` = (`B00000100`): Bitmask that when ANDed with [`statusFlags`](#statusflags) specifies whether there's currently an analog reading in progress (0 means there's no reading, any other value indicates a reading is in progress).
- `LAST_ANALOG_ADDRESS` = (`B00111000`): Bitmask that when ANDed with [`statusFlags`](#statusflags) and shifted 3 bits to the right specifies the most recent analog pin that a reading was taken from (0=A0, 1=A1, 2=A2, 3=A3, 4=A4, 5=A5, 6 and 7 are impossible values).
- `DIGITAL_1_ENABLED` =   (`B01000000`): Bitmask that when ANDed with [`statusFlags`](#statusflags) specifies whether the digital1 sensor is enabled for batch readings (0 means not enabled, any other value indicates its enabled).
- `DIGITAL_2_ENABLED` =   (`B10000000`): Bitmask that when ANDed with [`statusFlags`](#statusflags) specifies whether the digital2 sensor is enabled for batch readings (0 means not enabled, any other value indicates its enabled).
- `RESULT_READY` =        (`BB00000011`): Bitvalue that can be ORed with [`statusFlags`](#statusFlags) to specify that a single batch reading result is ready to be transferred into the data buffer and transmittied to the client.
- `ADMUX_PREFIX` =        (`(1<<REFS0)`): Bitwise value that should be used when setting the [`ADMUX`](#admux) register for taking analog readings. In general, `ADMUX` should be set as follows: `ADMUX = ADMUX_PREFIX | A#_ADDRESS`. This prefix sets the ADC converter chip to use the boards internal 5v power rail as it's reference voltage.
- `ADMUX_ADDRES` =   (`B00001111`): Bitmask that when ANDed with the [`ADMUX`](#admux) register evaluates to the address of the analog sensor a reading was most recently taken from. These addresses are equivalent to the `A#_ADDRESS` macros listed below.
- `A0_ADDRESS` =          (`0`): The multiplexer address of the A0 analog pin. ORing this value with an empty [`ADMUX`](#admux) register will set pin A0 as the target to take the next analog reading from.
- `A1_ADDRESS` =          (`1`): The multiplexer address of the A1 analog pin. ORing this value with an empty [`ADMUX`](#admux) register will set pin A1 as the target to take the next analog reading from.
- `A2_ADDRESS` =          (`2`): The multiplexer address of the A2 analog pin. ORing this value with an empty [`ADMUX`](#admux) register will set pin A2 as the target to take the next analog reading from.
- `A3_ADDRESS` =          (`3`): The multiplexer address of the A3 analog pin. ORing this value with an empty [`ADMUX`](#admux) register will set pin A3 as the target to take the next analog reading from.
- `A4_ADDRESS` =          (`4`): The multiplexer address of the A4 analog pin. ORing this value with an empty [`ADMUX`](#admux) register will set pin A4 as the target to take the next analog reading from.
- `A5_ADDRESS` =          (`5`): The multiplexer address of the A5 analog pin. ORing this value with an empty [`ADMUX`](#admux) register will set pin A5 as the target to take the next analog reading from.
- `ADC_START` =           (`(1<<ADEN)|(1<<ADSC)|(1<<ADIE)|(1<<ADPS2)`): Bitwise value that can be written into the ADCSRA register to start an analog to digital conversion. It keeps the ADC enabled, sets that it should start a conversion, enabled interrupting after the conversion finishes, and sets the ADC prescalar to 32.

## Global Variables
-------------------
The following list contains all the global variables used by the program. These are variables that can be accessed anywhere in the program.
Despite that statement however, only variables marked with `volatile` are safe to use in both the interrupt routines, and the normal program functions.
`volatile` is a keyword that makes any changes to a variable immediately visible to all of the program.
Otherwise the CPU might hold off on fully updating the variable for efficiency's sake.

All data types used in the program are of the form `uint#_t`. Where `uint` stands for unsigned integer, so it's a positive integer.
And the # part specifies how many bits long the data type is. So `uint32_t` is a 32 bit long positive integer.
These types are necessary to ensure we always know exactly how many bits each variable has and to help allocate memory more carefully.

The following is a list of every global variable used in the program, their type, and their default value:

- [`dataBuffer (uint8_t[])`](#databuffer): By default 60% of the board's available SRAM is allocated to this buffer. It's originally empty.
- [`dataStartPos (uint32_t)`](#databuffer): Starts at index 0.
- [`dataStopPos (uint32_t)`](#databuffer): Starts at index 0.
- [`statusFlags (uint8_t)`](#statusflags): Starts with every bit unset: B00000000.
- [`sensorFlags (uint8_t)`](#sensorflags): Starts with every bit unset: B00000000.
- [`sensorIDs (uint8_t[])`](#sensorids): 4 entries are allocated to the array (one for each sensor port). They all start out as 0.
- [`datastoreA (uint32_t)`](#datastore): This isn't set by default, since it's only ever used for temporary storage and isn't a _real_ variable.
- [`datastoreB (uint16_t)`](#datastore): This isn't set by default, since it's only ever used for temporary storage and isn't a _real_ variable.
- [`timestore (uint32_t)`](#timestore):

### dataBuffer:
This buffer stores the data gathered from sensors until it has a chance to be transmitted over the serial connection.
This is a *circular buffer*; normally a buffer has indexes between 0 and n, and data is always written starting at 0 possibly going up to n.
When the data in the buffer is transmitted, the entire buffer gets cleared and new data starts getting written at 0 again.
With a circular buffer, instead of having a fixed line of data from 0 to n, the buffer wraps around on itself. Indexes are taken '(mod n+1)' so once the end of the buffer is reached, the next byte is written to index 0 (the start of the buffer). It also doesn't matter where data starts in a circular buffer, and it need not always be at 0.
Circular buffers have a *valid data range*, a range of indexes where the unhandled data is stored, in this program that range is [`dataStartPos`,`dataStopPos`), and if `dataStopPos` is less than `dataStartPos` that just means the range wraps around past the end of the buffer.
The idea of doing it this way is that when new data is added to the buffer, `dataStopPos` is incremented (and possibly wrapped around mod n), and once data has been transmitted `dataStartPos` is incremented (and again maybe wrapped). By doing it this way we can only transmit some of the data and only move `dataStartPos` a little forward without having to move any data around, or actually delete anything. In a normal buffer, if we only wrote part of the data, then we'd have to shift all the buffer's data around so that index 0 would again contain the next byte of data to transmit.
So by using a circular buffer data can be continuously written and deleted from the buffer at the same time independently (since modifying the stop position doesn't affect the start position and vice verse), without ever needed to move any data around.
For more information on circular buffers, you can read about them [here](https://en.wikipedia.org/wiki/Circular_buffer)

Every sensor reading takes up exactly 10 bytes, the first 4 are a timestamp representing when the reading was started in microseconds since the program started running, and the next 6 contain all the sensor's data. It's more useful to view them as a single 48 bit long piece of data however, since the data doesn't match up cleanly with the boundaries of bytes. The bit-structure of the 6 data bytes are as follows:
- bits 0~9:   Value read from pin A0 (analog sensor 1 0~5v)
- bits 10~19: Value read from pin A1 (analog sensor 1 -10~+10v)
- bits 20~29: Value read from pin A2 (analog sensor 2 0~5v)
- bits 30~39: Value read from pin A3 (analog sensor 2 -10~+10v)
- bits 40~43: Values read from digital pins 2,3,4,5 (digital sensor 1)
- bits 44~47: Values read from digital pins 6,7,8,9 (digital sensor 2)

### statusFlags:
Bit array that holes flags for various settings, there's 8 bits in total for 8 settings. They are as follows:
- bits 0~1: The type of reading most recently taken (0 indicates a sensor ID scan, 1 indicates a non-batch sensor reasing, 2 indicates a batch reading, indicates a batch reading and that a reading has just finished (and hasn't been handled)).
- bit 2:    Whether there's currently a reading in progress (1 if there is, 0 otherwise).
- bits 3~5: The address of the analog pin a reading was most recently taken from (0 = A0, ..., 5=A5).
- bit 6:    Whether digital port 1 should be read from during batch readings (1 if it should be, 0 otherwise).
- bit 7:    Whether digital port 2 should be read from during batch readings (1 if it should be, 0 otherwise).

### sensorFlags:
Bit array that holds the states for each sensor port. There's 8 bits in total, the upper 4 keep track of which ports have sensors connected to them, and the lower 4 keep track of which sensors should be enabled during a batch reading. The `X_#_CONNECTED` and `X_#_ENABLED` macros are supplied 

The exact breakdown is as follows:
- bit 0: 1 if there's a sensor connected to analog port 1, 0 otherwise.
- bit 1: 1 if there's a sensor connected to analog port 2, 0 otherwise.
- bit 2: 1 if there's a sensor connected to digital port 1, 0 otherwise.
- bit 3: 1 if there's a sensor connected to digital port 2, 0 otherwise.
- bit 4: 1 if pin A0 is used by any of the connected analog sensors, 0 otherwise.
- bit 5: 1 if pin A1 is used by any of the connected analog sensors, 0 otherwise.
- bit 6: 1 if pin A2 is used by any of the connected analog sensors, 0 otherwise.
- bit 7: 1 if pin A3 is used by any of the connected analog sensors, 0 otherwise.

### sensorIDs:
This array stores the last recorded ID voltage of every port on the Vernier interface in MUX address order (analog1, analog2, digital1, digital2).
Every Vernier sensor has a dedicated connection on it with a set resistance that is the same for all sensors of the same model.
So by measuring the voltage across this connection, one can idenfity what kind of sensor is connected to that port.
Or if there's no voltage measured, then there's no sensor connected to that port.

When the processor has spare time, it scans through the ports one by one and measures their ID voltage.
It then compares the value to the corresponding entry stored in this array, and if a sufficient deviation is found, it informs the Arduino that the sensor must of changed, and triggers it to re-evaluate what kind of sensor is connected (and stores the new ID voltage).

When the program first starts up it does an initial scan over all the ports and writes their initial ID values into this array.

### datastore:
Both `datastoreA` and `datastoreB` are used to temporarily store a sensor reading before it gets written into the data buffer.
As mentioned in [`dataBuffer`](#databuffer), every sensor measurement is always 6 bytes, or 48 bits.
Hence the total size of both `datastoreA` and `datastoreB` come out to exactly this number.
The idea is that these are treated as one giant variable with all the bits of `datastoreA` slapped directly in front of all the bits of `datastoreB`, forming a single 48 bit long variable.

Since analog readings take time, the idea is that the datastore is wiped when a sensor reading starts, and as data is read from the various sensors it's written into the datastore.
After every reading has been taken, and there's nothing left to write into the datastore, only then are the contents of the data store broken into 6 individual bytes and copied into the data buffer.
This way we only update the data buffer a single time once all the data has been read, making it far simpler and safer to read and write the data buffer without any inherent threading checks.

The exact bit-structure of these variables is the same as the 6 byte data encoding disccussed in the [`dataBuffer`](#databuffer) section.

## Local Variables
------------------
Local variables are those that can only be accessed by a certain part of the program. They are either allocated `static` or not. Static means that the variable is shared through the entire program but only accessible within the scope that it was declared, whereas non-static variables are remade every time the function is called and deleted when the function is over. All local variables we use are `static` to save time that would of been wasted deleting and recreating variables every function call.

The `const` keyword means that it can't be modified after it's first declared.
For information on data-types, read the section on [Global Variables](#global-variables).

The following is a list of every local variable used in the program and their type:
- [`readingHandled (bool)`](#readinghandled): Variable that stores whether or not the analog reading has already been handled in the analog-to-digital interrupt service routine. This is used when iterating through each analog pin to decide which pin a reading should be taken from next; The first enabled pin we check where this value is already true must be next in line to be read.

## Functions
------------

