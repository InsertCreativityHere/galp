
package net.insertcreativity.galp;

/**
 * Base interface all sensors must implement. It provides basic methods for
 * retrieving data and information from the sensor.
**/
public interface Sensor
{
    /** Returns the name of the sensor. **/
    public String getName();

    /** Returns a short description about the sensor. **/
    public String getDesc();

    /** Performs a calibrated reading and returns the value. */
    public double getReading();

    /** Performs a reading without calibration and returns the value. */
    public double getReadingRaw();

    /** Calibrates the sensor with a known value.
        @param currentValue: The known value that the sensor should currently read.
        @return: Whether the calibration was successful. **/
    public boolean calibrate(double currentValue);
}
