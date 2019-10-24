
package net.insertcreativity.galp;

import java.io.Closeable;
import java.io.IOException;

/**
 * Base interface all sensors implement. It provides basic methods for interacting with the sensor.
**/
public interface Sensor extends Closeable
{
    /** Returns the name of the sensor. **/
    public String getName();

    /** Returns a short description about the sensor. **/
    public String getDesc();

    /** Returns the name of the variable that this sensor is measuring. **/
    public String getVariable();

    /** Returns the units that this sensor is calibrated to use. **/
    public String getUnits();

    /** Performs a calibrated reading and returns the value. **/
    public double getReading();

    /** Performs a reading without calibration and returns the value. **/
    public double getReadingRaw();

    /** Calibrates the sensor with a known value.
      * @param currentValue: The known value that the sensor should currently read.
      * @return: Whether the calibration was successful. **/
    public boolean calibrate(double currentValue);

    /** Closes the sensor and performs any necessary cleanup.
      * @throws IOException: If ann error occurs while closing. **/
    public void close() throws IOException;
}
