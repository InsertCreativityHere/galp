
//TODO HANDLE DERIVED DATA BUFFERS
//TODO TIGHTEN SCOPE VISIBILITY

package net.insertcreativity.galp;

import java.util.ArrayList;
import java.util.List;

/**
 * Class for encapsulating all the data of a session.
**/
public class Session
{
    // List of all the experiments run in this session.
    public List<String> experiments;
    // The display name of this session
    public String name;

    /** Creates a new session.
      * @param name: The name to display for this session. **/
    public Session(String name)
    {
        this.name = name;
        experiments = new ArrayList<String>();
    }

    /** Sets the name of this session. **/
    public void setName(String name)
    {
        this.name = name;
    }

    /** Returns the name of this session. **/
    public String getName()
    {
        return name;
    }
}
