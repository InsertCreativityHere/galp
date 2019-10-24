
package net.insertcreativity.galp;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

/**
 * Class for encapsulating all the data of an experiment.
**/
public class Experiment implements Serializable
{
    // List of all the trials run in this experiment.
    private final List<Trial> trials;
    // Array of all the sensors being used in this experiment.
    private final Sensor[] sensors;
    // Reference to the session this experiment is a part of.
    public final Session session;
    // The index of this experiment in it's session.
    public final int number;
    // The display name of this experiment, by default it's "Experiment #_" where '_' is the index number.
    private String name;

    /** Creates a new experiment.
      * @param parent: Reference to the session that created this experiment.
      * @param sensors: Array of the sensors being used in this experiment's trials.
      * @param num: The index of this experiment in it's session. **/
    public Experiment(Session parent, Sensor[] sensors, int num)
    {
        session = parent;
        this.sensors = sensors;
        number = num;
        name = "Experiment #" + number;
        trials = new ArrayList<Trial>();
    }

    /** Creates a new trial in this experiment and returns the corresponding trial object. **/
    public Trial newTrial()
    {
        Trial trial = new Trial(this, trials.size());
        trials.add(trial);
        return trial;
    }

    /** Returns the trial at the specified index.
      * @param index: The index of the trial to retrieve.
      * @return: The 'index'th trial done in this experiment.
      * @throws IndexOutOfBoundsException: If index is greater than or equal to 'getTrialCount', or negative. **/
    public Trial getTrial(int index)
    {
        return trials.get(index);
    }

    /** Returns the number of trials taken in this experiment so far **/
    public int getTrialCount()
    {
        return trials.size();
    }

    /** Returns a reference to this experiment's sensor array.
      * Modifications to either array will influence the other, so this method should be used catiously. **/
    @Deprecated
    public Sensor[] getSensors()
    {
        return sensors;
    }

    /** Sets the name of this experiment. **/
    public void setName(String name)
    {
        this.name = name;
    }

    /** Returns the name of this experiment. **/
    public String getName()
    {
        return name;
    }
}
