
package net.insertcreativity.galp;

/**
 * Base interface all sensors implement. It provides basic methods for
 * retrieving data and information from the sensor.
**/
public interface Sensor extends Closeable
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

    /** Closes the sensor and performs any necessary cleanup.
        @throws IOException: If ann error occurs while closing. **/
    public void close() throws IOException;
}
