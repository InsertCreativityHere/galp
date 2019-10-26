
package net.insertcreativity.galp;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

/**
 * Class for encapsulating all the data of a session.
**/
public class Session implements Serializable
{
    // List of all the experiments run in this session.
    private final List<Experiment> experiments;
    // The display name of this session
    private String name;

    /** Creates a new session.
      * @param name: The name to display for this session. **/
    public Session(String name)
    {
        this.name = name;
        experiments = new ArrayList<Experiment>();
    }

    /** Creates a new experiment in this session and returns the corresponding experiment object.
      * @param sensors: Array of all the sensors that are going to be used in each trial of the experiment.
      * @return: The newly created experiment object. **/
    protected Experiment newExperiment(Sensor[] sensors)
    {
        Experiment experiment = new Experiment(this, sensors, experiments.size());
        experiments.add(experiment);
        return experiment;
    }

    /** Returns the experiment at the specified index.
      * @param index: The index of the experiment to retrieve.
      * @return: The 'index'th experiment done in this session.
      * @throws IndexOutOfBoundsException: If index is greater than or equal to 'getExperimentCount', or negative. **/
    public Experiment getExperiment(int index)
    {
        return experiments.get(index);
    }

    /** Returns the number of experiments in this session. **/
    public int getExperimentCount()
    {
        return experiments.size();
    }

    /** Sets the name of this session. **/
    protected void setName(String name)
    {
        this.name = name;
    }

    /** Returns the name of this session. **/
    public String getName()
    {
        return name;
    }
}
