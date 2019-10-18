
//TODO add methods for loading/saving data from files.

package net.insertcreativity.galp;

import java.util.HashMap;
import java.util.Map;
import java.util.Random;

/**
 * Class for storing and managing data collected throughout sessions, and also for loading and saving data with files.
**/
public class DataManager
{
    // Map containing every data buffer in use by the program, keyed by a generated ID string, and stored as a Column.
    private final Map<String, Column> datastore;
    // The number of characters to generate for ID strings.
    private static final int IDlength;
    // The characters to generate ID strings from.
    private static final String allowed = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
    // Random number generator used for generating random ID strings.
    private final Random rand;

    /** Create a new Data Manager. **/
    public DataManager()
    {
        datastore = new HashMap<String, DoubleBuffer>();
        rand = new Random();
    }

    /** Registers a data buffer with this manager. The DataManager first wraps it in a column object to encapsulate the
      * name and units of the data, then it generates an ID for the column and stores a reference of the column with
      * the specified ID for later retrieval.
      * @param buffer: The buffer to register. The buffer is in no way altered by this method.
      * @param name: The display name of the buffer's data. This is used when displaying the data in the GUI.
      * @param units: The units of the buffer's data.
      * @return: The generated ID assigned to this buffer's column object. **/
    public String registerBuffer(DoubleBuffer buffer, String name, String units)
    {
        // Generate a random ID string.
        char[] IdBuilder = new char[IDlength];
        for(int i = 0; i < IDlength; i++)
        {
            IdBuilder[i] = allowed.charAt(rand.nextInt(allowed.length()));
        }
        String id = new String(IdBuilder);

        // Place the buffer in the datastore and return it's ID.
        datastore.put(id, new Column(buffer, name, units));
        return id;
    }

    /** Returns a reference to the column with the specified ID. **/
    public Column getColumn(String id)
    {
        return datastore.get(id);
    }

    /** Returns a reference to the buffer stored inside the column with the specified ID. **/
    public DoubleBuffer getBuffer(String id)
    {
        return datastore.get(id).data;
    }

    /**
     * Class for fully encapsulating the data collected in a buffer, including it's name and the units it's data is in.
    **/
    public class Column
    {
        // The display name for this column's data.
        private String name;
        // The stringified units this column's data is measured in.
        public final String units;
        // Reference to the underlying buffer storing this column's data.
        public final DoubleBuffer data;

        /** Creates a new Column with the specified name and units, backed by a provided buffer.
            @param buffer: The buffer the data is stored in.
            @param name: The name to use when displaying the data.
            @param units: The units that the data is stored in, stringified. **/
        public Column(DoubleBuffer buffer, String name, String units)
        {
            this.data = buffer;
            this.name = name;
            this.units = units;
        }

        /** Sets the name of this column. **/
        public void setName(String name)
        {
            this.name = name;
        }

        /** Returns the name of this coolumn. **/
        public String getName()
        {
            return name;
        }
    }
}
