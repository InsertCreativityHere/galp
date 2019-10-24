
//TODO add methods for loading/saving data from files.
//TODO add methods for exporting data to other programs.

package net.insertcreativity.galp;

import java.util.HashMap;
import java.util.Map;
import java.util.Random;

/**
 * Class for managing data. It provides a centralized location for storing all data collected throughout session, and
 * provides functions for saving/loading data with files, and exporting data into other applications.
 * Class for storing and managing data collected throughout sessions, and also for loading and saving data with files.
**/
public class DataManager
{
    // Map containing every data buffer currently in use by the program. Every data buffer is stored in a Column to
    // encapsulate descriptions and metadata alongside the buffer. The map keys are just longs.
    private final Map<Long, Column> datastore;
    // Map containing all the savefiles the program knows of. Any objects that have been loaded from a file, or any
    // objects that have been saved to a file will be stored in this map. It maps the object itself as keys to the
    // stringified absolute path of it's corresponding save file.
    private final Map<Object, String> savefiles;

    /** Create a new Data Manager. **/
    public DataManager()
    {
        datastore = new HashMap<Long, Column>();
        savefiles = new HashMap<Object, String>();
    }

    /** Allocates a new buffer and registers it in the Data Manager.
      * @param name: The display name for the data in the buffer.
      * @param units: The units that the buffer's data was measured in.
      * @param active: Whether the new buffer should be opened active (true) or closed (false)
      * @return: The ID number the Data Manager generates corresponding to the allocated buffer.
      *          This can be used to access the buffer in the future. **/
    public Long allocateBuffer(String name, String units, boolean active)
    {
        return registerBuffer(new DoubleBuffer(active), name, units);
    }

    /** Allocates a new buffer with the specified size and registers it in the Data Manager.
      * @param name: The display name for the data in the buffer.
      * @param units: The units that the buffer's data was measured in.
      * @param size: The amount to space to pre-allocate in the buffer.
      * @param active: Whether the new buffer should be opened active (true) or closed (false)
      * @return: The ID number the Data Manager generates corresponding to the allocated buffer.
      *          This can be used to access the buffer in the future. **/
    public Long allocateBuffer(String name, String units, int size, boolean active)
    {
        return registerBuffer(new DoubleBuffer(size, active), name, units);
    }

    public Long registerBuffer(DoubleBuffer buffer, String name, String units)
    {
        Column column = new Column(buffer, name, units);
        long id;
        synchronized(datastore)
        {
            id = datastore.size();
            datastore.put(id, column);
        }
        return id;
    }

    /** Returns the buffer that was registered with the corresponding ID, or null if no buffer was found. **/
    public DoubleBuffer getBuffer(long id)
    {
        Column column = datastore.get(id);
        if(column == null)
        {
            return null;
        } else {
            return getColumn(id).data;
        }
    }

    /** Returns the column object that was registered with the corresponding ID, or null if no column was found. **/
    public Column getColumn(long id)
    {
        return datastore.get(id);
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
