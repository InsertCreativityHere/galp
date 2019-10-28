
package net.insertcreativity.galp;

import java.io.Closeable;
import java.io.IOException;

/**
 * Class that encapsulates information and basic communication functionality for a sensor.
 * Every sensor must be connected to a Sensor Interface which provides the actual interfacing between this program and
 * the physical sensor. For fine-grained control or more complex data-capturing and communication the methods provided
 * by Sensor Interface should be used instead.
**/
public abstract class Sensor implements Closeable
{
    // The SensorInterace that this sensor is connected to.
    public final SensorInterface controller;
    // The display name of the sensor.
    public final String name;
    // A brief description of the sensor.
    public final String description;
    // The name of the variable that this sensor tracks (distance, force, radiation, etc...).
    public final String variable;
    // The stringified units that this sensor makes measurements in (M, S, Kg, NM, etc...).
    private String units;
    // Flag for whether the sensor is currently connected.
    private boolean connected;

    /** Creates a new sensor.
      * @param parent: The SensorInterface that this sensor is connected to.
      * @param name: The display name of the sensor.
      * @param desc: A brief description of the sensor.
      * @param var: The name of the variable that the sensor tracks.
      * @param units: The units that this sensor measures it's values in.
      * @param connected: whether or not the sensor is currently connected to it's interface and accessible. **/
    public Sensor(SensorInterface parent, String name, String desc, String var, String units, boolean connected)
    {
        controller = parent;
        this.name = name;
        description = desc;
        variable = var;
        this.units = units;
        this.connected = connected;
    }

    /** Returns the units that this sensor measures values in. **/
    public String getUnits()
    {
        return units;
    }

    /** Sets the units that the sensor measures it's readings in.
      * This should only be called by the sensor's corresponding interface. **/
    @Deprecated // This method shouldn't be used anywhere other than SensorInterface
    protected void setUnits(String units)
    {
        this.units = units;
    }

    /** Takes a reading from the sensor and returns the measured value. **/
    public double getReading()
    {
        return controller.getReading(this);
    }

    /** Takes an uncalibrated reading from the sensor and returns the measured value. **/
    public double getReadingRaw()
    {
        return controller.getReadingRaw(this);
    }

    /** Returns whether the sensor is currently connected and accessible. **/
    public boolean getConnected()
    {
        return connected;
    }

    /** Sets whether the sensor should report itself as being connected.
      * This should only be called by the sensor's corresponding interface. **/
    @Deprecated // This method shouldn't be used anywhere other than SensorInterface
    protected void setConnected(boolean connected)
    {
        this.connected = connected;
    }

    /** Calibrates the sensor by zeroing it's current value. The sensor will automatically adjust itself so it's
      * current value gets mapped to 0. **/
    protected void zero()
    {
       calibrate(0d);
    }

    /** Calibrates the specified sensor with a known current value it should be reporting.
      * @param currentValue: The current value that the sensor should be measuring. After calibration the sensor will
      *                      map whatever value it was reading before to the specified currentValue. **/
    protected abstract void calibrate(double currentValue);

    /** Adjusts a raw sensor reading by applying any necessary calibrations and accounting for different units too. **/
    public abstract double adjustReading(double value);

    /** Closes the sensor. This is used to perform any cleanup functions and let the sensor know it can power down.
      * @throws IOException: If an exception occurs while shutting down the sensor. **/
    public void close() throws IOException
    {
        try{
            controller.close(this);
        } finally {
            connected = false;
        }
    }
}
