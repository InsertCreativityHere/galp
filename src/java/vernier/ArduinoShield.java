
package net.insertcreativity.galp;

import com.fazecast.jSerialComm.SerialPort;

public class ArduinoShield implements SensorInterface, Runnable
{
    // Marker codes: These indicate the boundaries of packets sent/received between the client and the interface.
    public static final byte CLIENT_PACKET_START    = (byte)0b11001100;
    public static final byte CLIENT_PACKET_END      = (byte)0b10101010;
    public static final byte INTERFACE_PACKET_START = (byte)0b00110011;
    public static final byte INTERFACE_PACKET_END   = (byte)0b01010101;

    // Reserved callback codes: These are callback codes that are reserved and cannot be used for normal transmissions.
    public static final byte SENSOR_CHANGE_DETECTED = (byte)0b11111000;
    public static final byte BATCH_READING_STARTED  = (byte)0b11111001;
    public static final byte BATCH_READING_STOPPED  = (byte)0b11111010;
    public static final byte UNKNOWN_ERROR_OCCURED  = (byte)0b11111100;

    // Client command codes: These are used to transmit commands from the client to the Arduino interface.
    public static final byte GET_READING_COMMAND           = (byte)0b0000;
    public static final byte GET_READING_RAW_COMMAND       = (byte)0b0001;
    public static final byte START_BATCH_READING_COMMAND   = (byte)0b0010;
    public static final byte STOP_BATCH_READING_COMMAND    = (byte)0b0011;
    public static final byte SET_BUFFER_SIZE_COMMAND       = (byte)0b0100;
    public static final byte GET_BUFFER_SIZE_COMMAND       = (byte)0b0101;
    public static final byte SET_SAMPLE_PERIOD_COMMAND     = (byte)0b0110;
    public static final byte GET_SAMPLE_PERIOD_COMMAND     = (byte)0b0111;
    public static final byte IDENTIFY_SENSOR_COMMAND       = (byte)0b1000;
    public static final byte AUTO_SENSOR_DETECTION_COMMAND = (byte)0b1001;
    public static final byte CALIBRATE_SENSOR_COMMAND      = (byte)0b1010;
    public static final byte GET_CALIBRATION_DATA_COMMAND  = (byte)0b1011;
    public static final byte GET_NAME_COMMAND              = (byte)0b1100;
    public static final byte GET_INFORMATION_COMMAND       = (byte)0b1101;
    public static final byte SHUTDOWN_COMMAND              = (byte)0b1110;
    private static final byte UNMAPPED_COMMAND_16           = (byte)0b1111;

    // Address codes: There are used to specify the targets of commands issued by the client to the Arduino interface.
    public static final byte ARDUINO    = (byte)0b1111;
    public static final byte DIG_PORT_1 = (byte)0b0110;
    public static final byte DIG_PORT_2 = (byte)0b0101;
    public static final byte ANL_PORT_1 = (byte)0b0010;
    public static final byte ANL_PORT_2 = (byte)0b0001;

    public final String name;
    public final String desc;
    private SerialPort serialPort;
    private Sensor[] sensorPorts;
    private int baudrate;

    public ArduinoShield(SerialPort port, int baudrate)
    {
        this.serialPort = port;
        this.baudrate = baudrate;
    }

    public void run()
    {
        while(isRunning)
        {

        }
    }
}




[CLIENT_PACKET_START][CALLBACK_CODE][ADDRESS][COMMAND]<Payload>[CLIENT_PACKET_END]
[INTERFACE_PACKET_START][CALLBACK_CODE][ADDRESS]{[PAYLOAD_LENGTH]<Payload>}+[INTERFACE_PACKET_END]




// C>DISCVR
// A>DCLARE
// C>?INFO?
// A> send all the information over
// A>?INFO?
// C> send all the information over
// C>!READY
// A>READY!





    public final String serialPort;
    private final boolean[] sensorPortTypes;
    public final int digitalPortCount;
    public final int analogPortCount;
    public final int portCount;
    public final int baudrate;
    private Sensor[] sensors;

    public ArduinoInterface(boolean[] portTypes, String serialPort, int baudrate)
    {
        this.baudrate = baudrate;
        this.serialPort = serialPort;

        //TODO create the connection here!

        this.portCount = portTypes.length;
        this.sensors = new Sensor[portCount];
        this.sensorPortTypes = new boolean[portCount];
t
        dPortCount = 0;
        aPortCount = 0;
        for(int i = 0; i < portCount; i++)
        {
            this.sensorPortTypes[i] = portTypes[i];
            if(portTypes[i])
            {
                dPortCount++;
            } else {
                aPortCount++;
            }
        }
        this.digitalPortCount = dPortCount;
        this.analogPortCount = aPortCount;
    }

    public void run()
    {

    }
}
