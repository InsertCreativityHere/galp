
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

    /** Creates a new trial object.
      * @param parent: Reference to the experiment that created this trial.
      * @param bufferIDs: IDs for all of this trial's data buffers.
      * @param num: The index of this trial in it's experiment. **/
    @SuppressWarnings("deprecation")//We use 'experiment.getSensors', but don't modify it, so it's safe to do.
    public Trial(Experiment parent, int num)
    {
        experiment = parent;
        number = num;
        name = "Trial #" + number;

        // Allocate and register data buffers for the trial to store data in.
        Sensor sensors = experiment.getSensors();
        bufferIDs = new long[sensors.length];
        for(int i = 0; i < bufferIDs.length; i++)
        {
            bufferIDs[i] = DataManager.allocateBuffer(sensors[i].variable, sensors[i].units);
        }
    }

    /** Returns the bufferID at the specified index.
      * @param index: The index of the bufferID to retrieve.
      * @return: The 'index'th buffer being used by this trial.
      * @throws IndexOutOfBoundsException: If index is greater than or equal to 'getBufferCount', or negative. **/
    public long getBuffer(int index)
    {
        // Make sure the index is valid (less than the number of buffers and positive).
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

    /** Returns the number of buffers being used by this trial. **/
    public int getBufferCount()
    {
        return bufferIDs.length;
    }

    /** Sets the name of this trial. **/
    public void setName(String name)
    {
        this.name = name;
    }

    /** Returns the name of this trial. **/
    public String getName()
    {
        return name;
    }
}
