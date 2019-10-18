
package net.insertcreativity.galp;

/**
 * Base interface all sensor interfaces implement. It provides methods for accessing and communicating with both
 * the actual sensor interface and any sensors attached to it.
**/
public interface SensorInterface extends Closeable
{
    /** Returns the name of the interface. **/
    public String getName();

    /** Returns a short description about the interface. **/
    public String getDesc();

    /** Returns an object representing the sensor connected to the specified port.
        @param port: The port number to check.
        @return: Any sensor currently connected to the port, or null if there are no connected sensors. **/
    public Sensor getSensor(int port);
//TODO UPDATE TERMINOLOGY AND DOCS HERE DOWN
    /** Returns an array of the sensor objects for each port of the interface.
        @return: An array of sensor objects the same length as there are ports on the interface. For ports with a
                 sensor connected, it's sensor object is used . For unconnected ports, the entry is set to null. **/
    public Sensor[] getSensors();

    /** Returns whether or not this interface supports data buffering. **/
    public boolean supportsBuffering();

    /** Sets the internal buffer size the interface should use for a port, and returns how much
        buffer space was allocated. For interfaces that don't support buffering, this always returns -1.
        Interfaces are free to allocate sizes different from the specified amount if they wish, so the returns value
        of this method should always be checked.
        @param port: The port number to specify the buffer size for.
        @param size: Specifies the number of elements to allocate space for in the port's buffer. Buffers are always
                     double arrays, so the actual memory footprint will be (8*size) many bytes. A buffer size of 0
                     will disable buffering on the port.
        @return: The number of elements successfully allocated. Interfaces are free to allocate memory different from
                 the specified size for any number of reasons. So this method will always return the actual size of the
                 buffer after the method is finished. A value of 0 indicates that buffering was disabled, negative
                 values indicate an error occured in allocation, but the magnitude of the value still is the number of
                 elements allocated in the buffer. **/
    public int setBufferSize(int port, int size);

    /** Calibrates the sensor with a known value. **/
    public boolean calibrate(double currentValue);

    /** Returns a calibrated reading from the sensor on the specified port. This method will still returns a value
        even if no sensor is attached to the specified port. **/
    public double getReading(int port);

    /** Returns a raw (uncalibrated) reading from the sensor on the specified port. This method will still return a
        value even if no sensor is attached to the specified port. **/
    public double getReadingRaw(int port);

    /** Sets the sampling period on the specified port, this specifies how long to wait between consecutive readings
        on a sensor.
        @param port: The port to set the sampling period of.
        @param period: The number of microseconds to wait between consecutive samples are taken. Making this too small
                       can result in serious instabalities both in measured values, and the sensor hardware itself. In
                       some cases, the accuracy of this delay will be limited by the chronologistical resolution of the
                       interface and sensors in use. For instance, many interfaces may only support up to millisecond
                       resolutions.
        @return: The actual sample period set on the port. For reasons explained above, this may differ from the
                 specified period.**/
    public long setSamplingPeriod(int port, long period);

    /** Returns the current sampling period being used on the specified port. **/
    public long getSamplingPeriod(int port);

    /** Starts a batch reading on the specified port. If there is no sensor attached to the port, this immediately
        returns a closed buffer. Values will be read into the buffer once per sample period until manually stopped.
        @param port: The port to start the batch reading on.
        @return: Immediately returns a DoubleBuffer in streaming mode. The buffer will be updated with new values
                 as they're received from the sensor. When the reading is stopped, the buffer will be marked closed.**/
    public DoubleBuffer startBatchReading(int port);

    /** Starts a batch reading on the specified port. If there is no sensor attached to the port, this immediately
        returns a closed buffer. The specified number of values will be read from the sensor, one per sample period,
        unless the reading is prematurely manually stopped. Once count many readings have been taken, the buffer is
        marked as closed.
        @param port: The port to start the batch reading on.
        @param count: The number of readings to take.
        @return: Immediately returns a DoubleBuffer in streaming mode. The buffer will be updated with new values
                 as they're received from the sensor. When count many readings have been taken, or the batch has been
                 manually stopped, the buffer is marked as closed.**/
    public DoubleBuffer startBatchReading(int port, int count);

    /** Starts a batch reading on the specified port when the start trigger evaluates as true, and continues reading
        until either the end trigger evaluates as true, or the reading is manually stopped. If there is no sensor
        attached to the port, this immediately returns a closed buffer. A reading is taken from the sensor once per
        sample period, and when the end trigger evaluates true, the buffer is marked as closed.
        @param port: The port to start the batch reading on.
        @param start: Trigger condition that specifies when to start the readings.
        @param end: Trigger condition that specified when to end the readings.
        @return: Immediately returns a DoubleBuffer in streaming mode. The buffer will be updated with new values
                 as they're received from the sensor. When the end trigger evaluates true, or the batch has been
                 manually stopped, the buffer is marked as closed. **/
    public DoubleBuffer startBatchReading(int port, Trigger start, Trigger end);

    /** Manually terminates a batch reading. This will immediately cancel the session, regardless of any
        triggers or specified reading counts. Any already recorded data will still remain in it's buffer.
        @param port: The port to stop reading on.**/
    public void stopBatchReading(int port);

    /** Closes the sensor interface and performs any necessary cleanup.
        @throws IOException: If ann error occurs while closing. **/
    public void close() throws IOException;
}
