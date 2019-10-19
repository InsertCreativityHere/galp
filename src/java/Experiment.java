
//TODO HANDLE DERIVED DATA BUFFERS
//TODO TIGHTEN SCOPE VISIBILITY

package net.insertcreativity.galp;

import java.util.ArrayList;
import java.util.List;

/**
 * Class for encapsulating all the data of an experiment.
**/
public class Experiment
{
    // Reference to the session this experiment is a part of.
    public final Session session;
    // List of all the trials run in this experiment.
    public final List<Trial> trials;
    // The index of this experiment in it's session.
    public final int number;
    // The display name of this experiment, by default it's "Experiment #_" where '_' is the index number.
    private String name;

    /** Creates a new experiment.
      * @param parent: Reference to the session that created this experiment.
      * @param num: The index of this experiment in it's session. **/
    public Experiment(Session parent, int num)
    {
        session = parent;
        number = num;
        name = "Experiment #" + number;
        trials = new ArrayList<Trial>();
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
