
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

    /** Creates a new trial in this experiment and returns the corresponding trial object.
      * @return: The newly created trial object. **/
    protected Trial newTrial()
    {
        Trial trial = new Trial(this, trials.size());
        trials.add(trial);
        return trial;
    }

    /** Creates a new trial in this experiment and returns the corresponding trial object.
      * @param count: The number of data points expected to be recorded in the trial.
      * @return The newly created trial object. **/
    protected Trial newTrial(int count)
    {
        Trial trial = new Trial(this, trials.size(), count);
        trials.add(trial);
        return trial;
    }

    /** Returns the number of trials taken in this experiment so far **/
    public int getTrialCount()
    {
        return trials.size();
    }

    /** Returns the trial at the specified index.
      * @param index: The index of the trial to retrieve.
      * @return: The 'index'th trial done in this experiment.
      * @throws IndexOutOfBoundsException: If index is greater than or equal to 'getTrialCount', or negative. **/
    public Trial getTrial(int index)
    {
        return trials.get(index);
    }

    /** Returns the number of sensors being used in this experiment. **/
    public int getSensorCount()
    {
        return sensors.length;
    }

    /** Returns the sensor at the specified index.
      * @param index: The index of the sensor to retrieve.
      * @return: The 'index'th sensor being used in this experiment.
      * @throws IndexOutOfBoundsException: If the index is greater than or equal to 'getSensorCount', or negative. **/
    public Sensor getSensor(int index)
    {
        // Make sure the index is valid (less than the number of sensors and non-negative).
        if(index >= sensors.length)
        {
            throw new IndexOutOfBoundsException("Index '" + index + "' is out of bounds. Length='" + sensors.length + "'");
        } else
        if(index < 0)
        {
            throw new IndexOutOfBoundsException("Index '" + index + "' cannot be negative.");
        }
        return sensors[index];
    }

    /** Sets the name of this experiment. **/
    protected void setName(String name)
    {
        this.name = name;
    }

    /** Returns the name of this experiment. **/
    public String getName()
    {
        return name;
    }
}
