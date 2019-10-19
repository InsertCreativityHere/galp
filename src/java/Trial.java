
//TODO HANDLE DERIVED DATA BUFFERS
//TODO TIGHTEN SCOPE VISIBILITY

package net.insertcreativity.galp;

import java.io.Serializable;

/**
 * Class for encapsulating all the data of a trial.
**/
public class Trial implements Serializable;
{
    // Reference to the experiment this trial is a part of.
    public final Experiment experiment;
    // The IDs of all the data buffers data was collected into during this trial, as specified by the DataManager.
    // This will always be the same length as 'experiment.units'.
    public final String[] dataIDs;
    // The index of this trial in it's experiment.
    public final int number;
    // The display name of this trial, by default it's "Trial #_" where '_' is the index number.
    private String name;

    /** Creates a new trial object.
      * @param parent: Reference to the experiment that created this trial.
      * @param bufferIDs: IDs for all of this trial's data buffers.
      * @param num: The index of this trial in it's experiment. **/
    public Trial(Experiment parent, String[] bufferIDs, int num)
    {
        experiment = parent;
        dataIDs = bufferIDs;
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
}
