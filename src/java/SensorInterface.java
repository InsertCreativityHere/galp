
package net.insertcreativity.galp;

import java.io.Closeable;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * Base class all sensor interfaces extend. It specified the contracts and expected behavoir interfaces must provide
 * and includes the basic field and functionality necessary for any sensor interface along with some convenience
 * wrapper methods.
**/
public abstract class SensorInterface implements Closeable
{
    // The display name of the sensor interface.
    public final String name;
    // A brief description about the sensor interface.
    public final String description;
    // Flag indicating whether the interface is currently connected and usable.
    private boolean connected;
    // The period of time the interface should wait between consecutive readings during batch readings (in nanoseconds).
    private long samplingPeriod;
    // List of all the sensors connected to this interface. Values of null indicate that a sensor could be registered
    // at the specified index, however there's no sensor currently connected at that index. This is provided solely for
    // use by implementing subclasses and shouldn't be used or modified externally.
    @Deprecated // Direct modifications of the sensors can cause serious instabilities.
    protected final List<Sensor> sensors;
    // List of all the data buffers this sensor is the owner of. This list will always be the same length, and indexed
    // in the same order as sensors. So the ith buffer will contain data read from the ith sensor. These are only used
    // during batch readings, and is provided solely for use by implementing subclasses and shouldn't be used or
    // modified externally.
    @Deprecated // Direct modifications of the buffers can cause serious instabilities.
    protected final List<DoubleBuffer> buffers;

    /** Creates a new Sensor Interface.
      * @param name: The display name for the sensor interface.
      * @param desc: A brief description about the sensor interface.
      * @param connected: Whether the sensor interface is currently connected.
      * @param sensorCount:  **/
    public SensorInterface(String name, String desc, boolean connected, int sensorCount)
    {
        this.name = name;
        description = desc;
        this.connected = connected;

        sensors = new ArrayList<Sensor>(sensorCount);
        buffers = new ArrayList<DoubleBuffer>(sensorCount);
    }

    /** Returns the number of sensors this interface supports. If there is no limit to the number of sesnsors this
      * interface can simulataneously utilize, this returns -1. **/
    public int getSensorCount()
    {
        return sensors.size();
    }

    /** Returns the sensor at the specified index. Usually indexes are representative of port numbers for interfaces
      * that have dedicated sensor connection ports, but there is no strict contract for index meanings.
      * @param index: The index of the sensor to retrieve.
      * @return: A Sensor object representing the sensor assigned to the specified index, or null if no sensor is
      *          assigned. Note, indexes outside the range supported by the interface will throw
      *          IndexOutOfBoundsException instead. Null is only returned if the index is a valid sensor index, but
      *          no sensors are currently on that index.
      * @throws IndexOutOfBoundsException: If the provided index is outside the index range this interface supports. **/
    public Sensor getSensor(int index)
    {
        return sensors.get(index);
    }

    /** Retrieves the index that the specified sensor is assigned to, or -1 if the sensor isn't connected to this
      * interface. */
    public int getIndex(Sensor sensor)
    {
        return sensors.indexOf(sensor);
    }

    /** Returns an array of all the interface's sensors. For interfaces with a fixed number of sensor ports, the number
      * of ports will always match the length of the returned array. For indexes where no sensor is currently connected
      * it will instead have a value of null. The returned array allows obeys 'getSensors()[i]=getSensor(i)' for all
      * valid indexes. Note, changes to the returned array won't impact the interface's sensor list, and vice versa.
      * The returned array is a deep copy. As a result, using this method should be generally avoided. **/
    @Deprecated // This method makes unnecessary copies of data
    public Sensor[] getSensors()
    {
        return (Sensor[])sensors.toArray();
    }

    /** Manually adds a sensor to the interface, overriding and removing any sensors currently connected to the
      * specified index. This can be used to setup and use unsupported sensors or as a manual backup in case auto-
      * detection of sensors fails to find a sensor. If any sensor is currently assigned that index it will be closed
      * automatically before assigning the new sensor to it's index. This should not be called while readings are in
      * progress, although you can if desired.
      * @param sensor: The sensor to add to the interface. Often this will of had to of been constructed manually.
      * @param index: The index to assign this sensor too.
      * @throws IndexOutOfBoundsException: If the provided index is outside the range supported by this interface.
      * @throws IOException: If there's a problem communicating with the sensor interface. **/
    @Deprecated // Manually overriding a sensor when the sensor isn't actually connected is a bad idea.
    protected abstract void addSensor(Sensor sensor, int index) throws IOException;

    /** Triggers the interface to automatically rescan all it's sensors. This will check for and automatically update
      * any sensors that were removed from, or added to the interface. This should not be called while readings are in
      * progress, although you can if desired.
      * @throws IOException: If there's a problem communicating with the sensor interface. **/
    protected abstract void rescanSensors() throws IOException;

    /** Takes a reading from the sensor assigned to the specified index and returns the calibrated value it measured.
      * subclasses should override this method with a more efficient implementation.
      * @param index: The index of the sensor to take the reading from.
      * @return: a calibrated reading from the sensor.
      * @throws IllegalArgumentException: If there is no sensor currently attached at the specified index.
      * @throws IndexOutOfBoundsException: If the specified index is outside the range supported by this interface.
      * @throws IOException: If there's a problem communicating with the sensor interface. **/
    public double getReading(int index) throws IOException
    {
        double value = getReadingRaw(index);
        return sensors.get(index).adjustReading(value);
    }

    /** Takes a reading from the specified sensor and returns the calibrated value it measured.
      * Subclasses should override this method with a more efficient implementation.
      * @param sensor: The sensor to take the reading from.
      * @return: A calibrated reading from the sensor.
      * @throws IllegalArgumentException: If the provided sensor isn't connected to this interface.
      * @throws IOException: If there's a problem communicating with the sensor interface. **/
    public double getReading(Sensor sensor) throws IOException
    {
        return sensor.adjustReading(getReadingRaw(sensor));
    }

    /** Takes a reading from the sensor assigned to the specified index and returns the measured value uncalibrated
      * (this also means that any unit transformations won't be applied). Subclasses should override this method with a
      * more efficient implementation.
      * @param index: The index of the sensor to take the reading from.
      * @return: An uncalibrated reading from the sensor.
      * @throws IllegalArgumentException: If there is no sensor currently attached at the specified index.
      * @throws IndexOutOfBoundsException: If the specified index is outside the range supported by this interface.
      * @throws IOException: If there's a problem communicating with the sensor interface. **/
    public abstract double getReadingRaw(int index) throws IOException;

    /** Takes a reading from the specified sensor and returns the measured value uncalibrated (this also means that any
      * unit transformation won't be applied). Subclasses should override this method with a more efficient
      * implementation.
      * @param sensor: The sensor to take the reading from.
      * @return: An uncalibrated raw reading from the sensor.
      * @throws IllegalArgumentException: If the provided sensor isn't connected to this interface.
      * @throws IOException: If there's a problem communicating with the sensor interface. **/
    public double getReadingRaw(Sensor sensor) throws IOException
    {
        int index = sensors.indexOf(sensor);
        if(index == -1)
        {
            throw new IllegalArgumentException("Specified sensor isn't currently connected to this interface. sensor=" + sensor.name);
        } else {
            return getReadingRaw(index);
        }
    }

    /** Takes a reading from every sensor this interface has, and returns them in an array. For interfaces with a fixed
      * number of sensor ports, the array will always be the same length and in the same order as the ports. For ports
      * that don't have a sensor connected, NaN will be recorded instead of a value. Otherwise the calibrated value
      * measured by the sensor is recorded.
      * @return: An array of calibrated readings from every sensor this interface has.
      * @throws IOException: If there's a problem communicating with the sensor interface. **/
    public double[] getReadings() throws IOException
    {
        double[] values = getReadingsRaw();
        for(int i = 0; i < values.length; i++)
        {
            if(!Double.isNaN(values[i]))
            {
                values[i] = sensors.get(i).adjustReading(values[i]);
            }
        }
        return values;
    }

    /** Takes a reading from every sensor this interface has, and returns them in an array. For interfaces with a fixed
      * number of sensor ports, the array will always be the same length and in the same order as the ports. For ports
      * that don't have a sensor connected, NaN will be recorded instead of a value. Otherwise a raw uncalibrated value
      * measured by the sensor is recorded (this also means that any unit transformations won't be applied).
      * @return: An array of uncalibrated readings from every sensor this interface has.
      * @throws IOException: If there's a problem communicating with the sensor interface. **/
    public abstract double[] getReadingsRaw() throws IOException;

    /** Sets the sampling period to be used in batch readings. This is the period of time to wait in between taking
      * consecutive measurements. Making the sampling period too small can result in serious instabilites both in
      * the recorded data and the sensor hardware itself. Hence in cases where the interface deems the specified period
      * too small it's free to either set the period as it pleases and only use the requested value as a hint or throw
      * an exception indicating it can't support the requested period.
      * @param samplePeriod: The period to wait between consecutive measurements (in nanoseconds).
      * @return: The actual sampling period the interface set. The interface is free to only loosely use the requested
      *          period due to constraints on timing resolution and stability concerns it may possess.
      * @throws UnsupportedOperationException: If the interface physically cannot support the specified period.
      * @throws IOException: If there's a problem communicating with the sensor interface. **/
    protected long setSamplingPeriod(long samplePeriod) throws UnsupportedOperationException, IOException
    {
        samplingPeriod = samplePeriod;
        return samplePeriod;
    }

    /** Returns the sampling period this interface is currently using. **/
    public long getSamplingPeriod()
    {
        return samplingPeriod;
    }

    /** Attaches a data buffer to the specified sensor that it can use for batch readings. Data values measured during
      * the batch reading will be written into the buffer in real-time. If the buffer is null, then the sensor will be
      * disabled for the duration of any batch readings and no data will be recorded from it. If there is already a
      * data buffer attached to the sensor, that buffer will be closed and replaced by this one.
      * @param index: The index of the sensor to attach the buffer to.
      * @param buffer: The data buffer to attach to the sensor; if null then the sensor won't be used for any batch
      *                readings until a non-null buffer is attached again.
      * @throws IllegalArgumentException: If there is no sensor currently attached at the specified index.
      * @throws IndexOutOfBoundsException: If the specified index is outside the range supported by this interface. **/
    protected void setBatchBuffer(int index, DoubleBuffer buffer)
    {
        if(sensors.get(index) == null)
        {
            throw new IllegalArgumentException("There is no sensor currently attached at index " + index);
        }
        DoubleBuffer oldBuffer = buffers.set(index, buffer);
        if(oldBuffer != null)
        {
            oldBuffer.close();
        }
    }

    /** Attaches a data buffer to the specified sensor that it can use for batch readings. Data values measured during
      * the batch reading will be written into the buffer in real-time. If the buffer is null, then the sensor will be
      * disabled for the duration of any batch readings and no data will be recorded from it. If there is already a
      * data buffer attached to the sensor, that buffer will be closed and replaced by this one.
      * @param sensor: The sensor to attach the buffer to.
      * @param buffer: The data buffer to attach to the sensor; if null then the sensor won't be used for any batch
      *                readings until a non-null buffer is attached again.
      * @throws IllegalArgumentException: If the provided sensor isn't connected to this interface. **/
    protected void setBatchBuffer(Sensor sensor, DoubleBuffer buffer)
    {
        int index = sensors.indexOf(sensor);
        if(index == -1)
        {
            throw new IllegalArgumentException("Specified sensor isn't currently connected to this interface. sensor=" + sensor.name);
        } else {
            setBatchBuffer(index, buffer);
        }
    }

    /** Attaches data buffers to the specified sensors that they can use for batch readings. Data values measured
      * during the batch reading will be written into the buffer in real-time. If the buffer is null, then the sensor
      * will be disabled for the duration of any batch readings and no data will be recorded from it. If there is
      * already a data buffer attached to the sensor, that buffer will be closed and replaced by this one.
      * @param indexes: Array of indexes specifying which sensor to set which buffer to.
      * @param dataBuffers: The data buffers to attach to the sensors. Buffers are attached in order so dataBuffers[i]
      *                     will be attached to the sensor with index indexes[i]. If null, then the sensor won't be
      *                     used for any batch readings until a non-null buffer is attached again.
      * @throws IllegalArgumentException: If any of the indexes are valid but there aren't any sensors connected at it.
      * @throws IndexOutOfBoundsException: If any of the indexes are outside the range supported by this interface. **/
    protected void setBatchBuffers(int[] indexes, DoubleBuffer[] dataBuffers)
    {
        for(int i = 0; i < indexes.length; i++)
        {
            setBatchBuffer(indexes[i], dataBuffers[i]);
        }
    }

    /** Attaches data buffers to the specified sensors that they can use for batch readings. Data values measured
      * during the batch reading will be written into the buffer in real-time. If the buffer is null, then the sensor
      * will be disabled for the duration of any batch readings and no data will be recorded from it. If there is
      * already a data buffer attached to the sensor, that buffer will be closed and replaced by this one.
      * @param sensors: Array of all the sensors to set the buffers for.
      * @param dataBuffers: The data buffers to attach to the sensors. Buffers are attached in order so dataBuffers[i]
      *                     will be attached to sensors[i]. If null, then the sensor won't be used for any batch
      *                     readings until a non-null buffer is attached again.
      * @throws IllegalArgumentException: If any of the provided sensors aren't connected to this interface. **/
    protected void setBatchBuffers(Sensor[] sensors, DoubleBuffer[] dataBuffers)
    {
        for(int i = 0; i < sensors.length; i++)
        {
            setBatchBuffer(sensors[i], dataBuffers[i]);
        }
    }

    /** Detaches and closes any data buffers currently held by any sensors connected to this interface. **/
    protected void closeBatchBuffers()
    {
        for(int i = 0; i < buffers.size(); i++)
        {
            setBatchBuffer(i, null);
        }
    }

    /** Starts a batch reading. Once per sample period, a value is read from every sensor with a data buffer backing it
      * and written into said data buffer. This continues until manually stopped by a call to 'stopBatchReading'. **/
    protected abstract void startBatchReading() throws IOException;

    /** Starts a batch reading. Once per sample period, a value is read from every sensor with a data buffer backing it
      * and written into said data buffer. The batch reading will end after 'sampleCount' many sample periods have
      * passed, or it's manually stopped by a call to 'stopBatchReading'. **/
    protected abstract void startBatchReading(int sampleCount) throws IOException;

    /** Schedules a batch reading which will start when the start condition is triggered and will continue until the
      * end condition is triggered. During this time, once per sample period, a value is read from every sensor with a
      * data buffer backing it and written into said ata buffer. This ends either when the end condition is triggered
      * or it's manually stopped by a call to 'stopBatchReading'. **/
    protected abstract void startBatchReading(Trigger start, Trigger stop) throws IOException;

    /** Immediately stops any running batch readings regardless of their normal ending condition. **/
    protected abstract void stopBatchReading() throws IOException;

    /** Closes the sensor interface, any sensors still connected to it, and any associated buffers that are still open.
      * Subclasses should override this to perform any necessary cleanup of their own.
      * @throws IOException: If there's a problem communicating with or closing the sensor interface. This exception
      *                      should only be thrown for issues arising from closing the sensor interface itself.
      *                      Exceptions from closing individual sensors or subcomponents should be passed to
      *                      'Main.handleException'. This ensures everything has a chance to be closed. **/
    public void close() throws IOException
    {
        // Close all the sensors still attached to this interface.
        for(Sensor sensor : sensors)
        {
            if(sensor != null)
            {
                try {
                    close(sensor);
                } catch(Exception exception)
                {
                    Main.handleException("closing", exception);
                }
            }
        }
        // Close all the buffers still associated with this interface.
        for(DoubleBuffer buffer : buffers)
        {
            if(buffer != null)
            {
                buffer.close();
            }
        }
    }
    /** Closes the specified sensor and any buffers associeated with it that are still open.
      * Subclasses should override this to perform any additional cleanup of their own, and to notify the interface of
      * the sensor's closure.
      * @throws IOException: If there's a problem communicating with or closing the sensor hardware.
      * @throws IllegalArgumentException: If the provided sensor isn't connected to this interface. **/
    protected abstract void close(Sensor sensor) throws IOException
    {
        int index = sensors.indexOf(sensor);
        if(index == -1)
        {
            throw new IllegalArgumentException("Specified sensor isn't currently connected to this interface. sensor=" + sensor.name);
        } else {
            if(buffers[i] != null)
            {
                buffers[i].close();
            }
        }
    }
}
