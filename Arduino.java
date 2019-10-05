
package net.insertcreativity.galp;

public class ArduinoInterface implements Runnable
{
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
