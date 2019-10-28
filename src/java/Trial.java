
package net.insertcreativity.galp;

import java.io.Serializable;

/**
 * Class for encapsulating all the data of a trial.
**/
public class Trial implements Serializable
{
    // The IDs for each data buffer that this trial uses for storing it's data, as specified by DataManager.
    private final long[] bufferIDs;
    // Reference to the experiment this trial is a part of.
    public final Experiment experiment;
    // The index of this trial in it's experiment.
    public final int number;
    // The display name of this trial, by default it's "Trial #_" where '_' is the index number.
    private String name;

    /** Creates a new trial object. Data buffers are automatically allocated and registered for the trial.
      * @param parent: Reference to the experiment that created this trial.
      * @param num: The index of this trial in it's experiment. **/
    public Trial(Experiment parent, int num)
    {
        experiment = parent;
        number = num;
        name = "Trial #" + number;

        bufferIDs = new long[experiment.getSensorCount()];
        for(int i = 0; i < bufferIDs.length; i++)
        {
            Sensor sensor = experiment.getSensor(i);
            bufferIDs[i] = Main.dataManager.allocateBuffer(sensor.variable, sensor.getUnits(), true);
        }
    }

    /** Creates a new trial object. Data buffers are automatically allocated and registered for the trial.
      * @param parent: Reference to the experiment that created this trial.
      * @param num: The index of this trial in it's experiment.
      * @param count: The number of data points expected to be recorded by the trial. This is used while constructing
      *               the trial's data buffers. **/
    public Trial(Experiment parent, int num, int count)
    {
        experiment = parent;
        number = num;
        name = "Trial #" + number;

        bufferIDs = new long[experiment.getSensorCount()];
        for(int i = 0; i < bufferIDs.length; i++)
        {
            Sensor sensor = experiment.getSensor(i);
            bufferIDs[i] = Main.dataManager.allocateBuffer(sensor.variable, sensor.getUnits(), count, true);
        }
    }

    /** Creates a new trial object with an array of pre-registered buffers.
      * @param parent: Reference to the experiment that created this trial.
      * @param bufferIDs: IDs for all of this trial's data buffers.
      * @param num: The index of this trial in it's experiment. **/
    public Trial(Experiment parent, long[] bufferIDs, int num)
    {
        experiment = parent;
        number = num;
        name = "Trial #" + number;
        this.bufferIDs = bufferIDs;
    }

    /** Returns the number of buffers being used by this trial. **/
    public int getBufferCount()
    {
        return bufferIDs.length;
    }

    /** Returns the bufferID at the specified index.
      * @param index: The index of the bufferID to retrieve.
      * @return: The 'index'th buffer being used by this trial.
      * @throws IndexOutOfBoundsException: If index is greater than or equal to 'getBufferCount', or negative. **/
    public long getBuffer(int index)
    {
        // Make sure the index is valid (less than the number of buffers and non-negative).
        if(index >= bufferIDs.length)
        {
            throw new IndexOutOfBoundsException("Index '" + index + "' is out of bounds. Length='" + bufferIDs.length + "'");
        } else
        if(index < 0)
        {
            throw new IndexOutOfBoundsException("Index '" + index + "' cannot be negative.");
        }
        return bufferIDs[index];
    }

    /** Returns a direct reference to the buffer ids being used by this trial. Alterations to this array will affect
      * the trial's array too, so this method should be used with extreme caution. **/
    @Deprecated // Altering the returned array can have serious consequences on the trial.
    public long[] getBuffers()
    {
        return bufferIDs;
    }

    /** Sets the name of this trial. **/
    protected void setName(String name)
    {
        this.name = name;
    }

    /** Returns the name of this trial. **/
    public String getName()
    {
        return name;
    }
}
