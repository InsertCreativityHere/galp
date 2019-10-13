
package net.insertcreativity.galp;

/**
 * Class for encapsulating all the data needed to describe a trial.
**/
public class Trial
{
    // Reference to the experiment this trial is a part of.
    public final Experiment experiment;
    // The IDs of any buffers storing data from this trial, as specified by the DataManager.
    public final String[] dataID;
    // The index of this trial in it's experiment.
    public final int number;
    // The display name of this trial, by default it's "Trial #_" where '_' is the trial number.
    private String name;

    /** Creates a new trial object.
        @param parent: Reference to the experiment that created this trial.
        @param dataID: IDs for any of this trial's data buffers.
        @param num: The index of this in it's experiment. **/
    public Trial(Experiment parent, String[] bufferID, int num)
    {
        experiment = parent;
        dataID = bufferID;
        number = num;
        name = "Trial #" + number;
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

    /** Returns one of the trial's data buffers. **/
    public DoubleBuffer getData(int index)
    {
        return DataManager.getBuffer(dataID[index]);
    }
}
