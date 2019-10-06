
package net.insertcreativity.galp;

/**
 * TODO: mention streaming mode
 * TODO: mention not thread safe
**/
public class DoubleBuffer
{
    // The default number of elements to allocate in the buffer if none are specified.
    private static final int DEFAULT_BUFFER_ALLOCATE_AMOUNT = 256;
    // The default number of elements to grow the buffer by if none were specified.
    private static final int DEFAULT_BUFFER_GROW_AMOUNT = 128;
    // Flag indicating whether or not the buffer is still in streaming mode.
    private boolean closed;
    // Array containing the actual data this buffer encapsulates.
    private double[] data;
    // The number of elements currently stored in the buffer.
    private int count;

    /** Creates a new buffer for holding primitive doubles, with the default allocation size and that's opened in
        streaming mode. **/
    public DoubleBuffer()
    {
        this(DEFAULT_BUFFER_ALLOCATE_AMOUNT, true);
    }

    /** Creates a new buffer for holding primitive doubles, with the default allocation size.
        @param streaming: Whether the buffer should be opened in streaming mode or not. **/
    public DoubleBuffer(boolean streaming)
    {
        this(DEFAULT_BUFFER_ALLOCATE_AMOUNT, streaming);
    }

    /** Creates a new buffer for holding primitive doubles.
        @param length: The number of doubles to pre-allocate space for in the buffer.
        @param streaming: Whether the buffer should be opened in streaming mode or not. **/
    public DoubleBuffer(int length, boolean streaming)
    {
        data = new double[length];
        closed = !streaming;
        count = 0;
    }

    /** Creates a new buffer for holding primitive doubles that contains the specified array.
        @param d: An array of doubles to copy into the buffer. The buffer is allocated to be the exact size needed to
                  hold the provided array. The provided array isn't altered by this constructor.
        @param streaming: Whether the buffer should be opened in streaming mode or not. **/
    public DoubleBuffer(double[] d, boolean streaming)
    {
        this(d, 0, d.length, streaming);
    }

    /** Creates a new buffer for holding primitive doubles that contains a subsection of the specified array.
        @param d: An array holding doubles to copy into the buffer.
        @param offset: The index to start copying doubles into the buffer from. No elements before this index will be
                       stored in the buffer.
        @param length: The number of elements to store in the buffer. The buffer is allocated to be the exact size
                       needed to hold length many doubles.
        @param streaming: Whether the buffer should be opened in streaming mode or not.
        @throws IndexOutOfBoundsException: If offset+length is larger than the length of the provided array. **/
    public DoubleBuffer(double[] d, int offset, int length, boolean streaming)
    {
        this(length, streaming);

        // Ensure the range is valid.
        if((offset + length) > d.length)
        {
            throw new IndexOutOfBoundsException("Specified range is outside the bounds of the provided array. array.length='" + d.length + "', range=(" + offset + "," + (range+length) + ")'.");
        }
        // Copy the values into the buffer.
        System.arraycopy(d, offset, data, count, length);
        count += length;
    }

    /** Appends the provided value at the end of the buffer, growing the buffer if necessary. **/
    public void append(double d)
    {
        // Grow the size of the buffer if there isn't enough space.
        if(count + 1 >= data.length)
        {
            grow(DEFAULT_BUFFER_GROW_AMOUNT);
        }
        // Store the data value and increment the number of values stored in the buffer.
        data[count++] = d;
    }

    /** Appends the entire array at the end of the buffer, growing the buffer if necessary. */
    public void append(double[] d)
    {
        append(d, 0, d.length);
    }

    /** Appends a subsection of the specified array at the end of the buffer, growing the buffer if necessary.
        @param d: An array holding the doubles to append.
        @param offset: The index to starting appending doubles into the buffer from. No elements before this index will
                       be stored in the buffer.
        @param length: The number of elements to append.
        @throws IndexOutOfBoundsException: If offset+length is larger than the length of the provided array, or either
                                           the offset or length are negative. **/
    public void append(double[] d, int offset, int length)
    {
        // Ensure the range is valid.
        if((offset + length) > d.length)
        {
            throw new IndexOutOfBoundsException("Specified range is outside the bounds of the provided array. array.length='" + d.length + "', range=(" + offset + "," + (offset+length) + ")'.");
        } else
        if(offset < 0)
        {
            throw new IndexOutOfBoundsException("Cannot have negative offset: '" + offset + "'");
        } else
        if(length < 0)
        {
            throw new IndexOutOfBoundsException("Cannot have negative length: '" + length + "'");
        }
        // Grow the size of the buffer if there isn't enough space.
        int space = data.length - count - length;
        if(space < 0)
        {
            // Only grow the buffer just enough to fit the extra values.
            grow(-space);
        }
        // Copy the values into the buffer.
        System.arraycopy(d, offset, data, count, length);
        count += length;
    }

    /** Inserts the provided value at the specified index.
        @param d: The value to insert into the buffer.
        @param index: The position to insert the value in the buffer.
        @throws IndexOutOfBoundsException: If the index is negative or larger than the size of the buffer.**/
    public void insert(double d, int index)
    {
        // Make sure the index is valid (less than the number of elements and positive).
        if(index >= count)
        {
            throw new IndexOutOfBoundsException("Index '" + index + "' is out of bounds. Length='" + count + "'");
        } else
        if(index < 0)
        {
            throw new IndexOutOfBoundsException("Index '" + index "' cannot be negative.");
        }
        // Grow the size of the buffer if there isn't enough space.
        if(count + 1 >= data.length)
        {
            grow(DEFAULT_BUFFER_GROW_AMOUNT);
        }
        // Shift every element in the buffer forward by 1, starting at index.
        System.arraycopy(data, index, data, (index+1), (count - index - 1));
        // Store the data value and increment the number of values stored in the buffer.
        data[index] = d;
        count++;
    }

    /** Returns the value at the specified index.
        @param index: The position to retrieve the value from.
        @return: The value at the specified index.
        @throws IndexOutOfBoundsException: If the index is negative or larger than the size of the buffer.**/
    public double get(int index)
    {
        // Make sure the index is valid (less than the number of elements and positive).
        if(index >= count)
        {
            throw new IndexOutOfBoundsException("Index '" + index + "' is out of bounds. Length='" + count + "'");
        } else
        if(index < 0)
        {
            throw new IndexOutOfBoundsException("Index '" + index "' cannot be negative.");
        }
        // Return the requested value.
        return data[index];
    }

    /** Returns a copy of all the data currently in the buffer. **/
    public double[] getData()
    {
        // Create and return a copy of all the data currently in the buffer.
        double[] dataCopy = new double[count];
        System.arraycopy(data, 0, dataCopy, 0, count);
        return dataCopy;
    }

    /** Removes an element from the buffer and returns it.
        @param index: The index of the element to remove.
        @return: The element that was removed.
        @throws IndexOutOfBoundsException: If the index is negative or larger than the size of the buffer.**/
    public double remove(int index)
    {
        // Make sure the index is valid (less than the number of elements and positive).
        if(index >= count)
        {
            throw new IndexOutOfBoundsException("Index '" + index + "' is out of bounds. Length='" + count + "'");
        } else
        if(index < 0)
        {
            throw new IndexOutOfBoundsException("Index '" + index "' cannot be negative.");
        }
        // Shift every element in the buffer back by 1, starting at index.
        System.arraycopy(data, (index+1), data, index, (count - index - 1));
        count--;
    }

    /** Removes a range of elements from the buffer.
        @param index: The index to start removing elements from.
        @param length: The number of elements to remove, starting with the element at index.
        @throws IndexOutOfBoundsException: If offset+length is larger than the length of the provided array, or either
                                           the offset or length are negative. **/
    public void remove(int index, int length)
    {
        // Ensure the range is valid.
        if((index + length) > d.length)
        {
            throw new IndexOutOfBoundsException("Specified range is outside the bounds of the provided array. array.length='" + d.length + "', range=(" + index + "," + (index+length) + ")'.");
        } else
        if(index < 0)
        {
            throw new IndexOutOfBoundsException("Cannot have negative index: '" + index + "'");
        } else
        if(length < 0)
        {
            throw new IndexOutOfBoundsException("Cannot have negative length: '" + length + "'");
        }
        // Shift every element in the buffer back by count, starting at index.
        System.arraycopy(data, (index+length), data, index, (count - index - length));
        count -= length;
    }

    /** Expands the underlying array this buffer uses by the specified amount. This doesn't affect the buffered data.
        @param growBy: The number of elements to expand the backing array by.
        @return: The new length of the backing array.
        @throws IllegalArgumentException: If growBy is negative. **/
    public int grow(int growBy)
    {
        if(growBy < 0)
        {
            throw IllegalArgumentException("Cannot grow buffer by a negative length: '" + growBy + "'.");
        }

        // Allocate a new buffer with the required size.
        double[] newBuffer = new double[data.length + growBy];
        // Copy any data in the old buffer into the new one.
        System.arraycopy(data, 0, newBuffer, 0, count);
        data = newBuffer;
        return data.length;
    }

    /** Shrinks the underlying array this buffer uses by the specified amount. Any data that doesn't fit in the new
        buffer will be discarded.
        @param shrinkBy: The number of elements to shrink the backing array by.
        @return: The new length of the backing array.
        @throws IllegalArgumentException: If shrinkBy is negative. **/
    public int shrink(int shrinkBy)
    {
        // Allocate a new buffer with the required size.
        double[] newBuffer = new double[data.length - shrinkBy];
        // Copy any data in the old buffer into the new one that fits.
        newCount = Math.min(count, newBuffer.length);
        System.arraycopy(data, 0, newBuffer, 9, newCount);
        data = newBuffer;
        count = newCount;
        return data.length;
    }

    /** Returns the number of elements currently stored in the buffer. Note this is not the actual size of the buffer,
        which is often over-allocated for optimization. **/
    public int length()
    {
        return count;
    }

    /** Returns whether or not the buffer is closed. This is used in streaming mode to indicate that the object that
        was writing into the buffer has stopped (the stream has closed). **/
    public boolean isClosed()
    {
        return closed;
    }

    /** Marks the buffer as being closed. This should only be called by sources that were writing into the buffer to
        indicate they're finished writing. Any threads waiting on the doublebuffer object will be notified. **/
    public void close()
    {
        closed = true;
        synchronized(this){
            this.notifyAll();
        }
    }
}
