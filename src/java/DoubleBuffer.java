
package net.insertcreativity.galp;

import java.io.Serializable;

/**
 * This class provides a resizable buffer for storing primitive doubles in a fast and relatively efficient manner.
 * Note that while the buffer will auto-grow (allocate more memory as more elements are added), its not auto-shrinking,
 * meaning even if the buffer is fully emptied, without an explicit call to 'shrink' the buffer will retain it's old
 * size. While this class provides some basic methods with thread awareness, the implementation provides no internal
 * thread safety whatsoever, any necessary synchronization will need to be handled externally. This can be easily
 * acheived by having any threads that read or write to the buffer synchronize on it first, although usually thread
 * safety isn't necessary, and if data consistency is important, it's often worth just waiting for the buffer to close.
 *
 * Internally within this project the buffer is always used in one of two modes: 'active' and 'closed', and always
 * follows the contract that once an active buffer is closed, no more data will be added to the buffer. Some buffers
 * are closed at creation, when data is loaded from a file for example, and there's no more data to be written after
 * construction. Other buffers are returned 'active', where data is actively written to the buffer after construction.
 * Every active buffer has exactly one owner, the sole object allowed to modify it's state or close it. This is usually
 * a Sensor or Sensor interface, which puts readings into the buffer then closes it after the readings have been taken.
**/
public class DoubleBuffer implements Cloneable, Serializable
{
    // The default number of elements to allocate in the buffer if none are specified.
    private static final int DEFAULT_BUFFER_ALLOCATE_AMOUNT = 256;
    // The default number of elements to grow the buffer by if none were specified.
    private static final int DEFAULT_BUFFER_GROW_AMOUNT = 128;
    // Flag indicating whether or not the buffer is still in streaming mode.
    private boolean closed;
    // Array containing the actual data backing this buffer.
    private double[] data;
    // The number of elements currently stored in the buffer.
    private int count;

    /** Creates an active buffer for holding primitive doubles, with the default allocation size. **/
    public DoubleBuffer()
    {
        this(DEFAULT_BUFFER_ALLOCATE_AMOUNT, true);
    }

    /** Creates a new buffer for holding primitive doubles, with the default allocation size.
      * @param streaming: True if the buffer should be left active after construction, or false to close it. **/
    public DoubleBuffer(boolean active)
    {
        this(DEFAULT_BUFFER_ALLOCATE_AMOUNT, active);
    }

    /** Creates a new buffer for holding primitive doubles.
      * @param size: The number of doubles to pre-allocate space for in the buffer.
      * @param active: True if the buffer should be left active after construction, or false to close it. **/
    public DoubleBuffer(int size, boolean active)
    {
        data = new double[size];
        closed = !active;
        count = 0;
    }

    /** Creates a new buffer for holding primitive doubles that contains the specified array.
      * @param d: An array of doubles to copy into the buffer. The buffer is allocated to be the exact size needed to
      *           hold the provided array. The provided array isn't altered by this constructor, and is copied into the
      *           buffer, so alterations to one array will not affect the other.
      * @param active: True if the buffer should be left active after construction, or false to close it. **/
    public DoubleBuffer(double[] d, boolean active)
    {
        this(d, 0, d.length, active);
    }

    /** Creates a new buffer for holding primitive doubles that contains a subsection of the specified array.
      * @param d: An array holding doubles to copy into the buffer. The buffer is allocated to be the exact size needed
      *           to hold the provided array. The provided array isn't altered by this constructor, and is copied into
      *           the buffer, so alterations to one array will not affect the other.
      * @param offset: The index to start copying doubles into the buffer from. No elements before this index will be
      *                stored in the buffer.
      * @param length: The number of elements to store in the buffer. The buffer is allocated to be the exact size
      *                needed to hold length many doubles.
      * @param active: True if the buffer should be left active after construction, or false to close it.
      * @throws IndexOutOfBoundsException: If offset+length is larger than the length of the provided array. **/
    public DoubleBuffer(double[] d, int offset, int length, boolean active)
    {
        this(length, active);

        // Ensure the range is valid.
        if((offset + length) > d.length)
        {
            throw new IndexOutOfBoundsException("Specified range is outside the bounds of the provided array. array.length='" + d.length + "', range=(" + offset + "," + (offset+length) + ")'.");
        }
        // Copy the values into the buffer.
        System.arraycopy(d, offset, data, count, length);
        count += length;
    }

    /** Returns a deep copy of this buffer. The copy contains all the data currently in the buffer, and is identical to
      * this buffer, except the clone is always returned closed, so any additional data that is written to this buffer
      * will not appeat in the clone. In general, alterations to either buffer will not affect the other. This is a true
      * deep copy. Note that cloning a buffer that isn't closed can sometimes cause unexpected results. **/
    public DoubleBuffer clone()
    {
        return new DoubleBuffer(this.data, false);
    }

    /** Returns a deep copy of this buffer. The copy contains all the data currently in the buffer, and is returned in
      * the specified mode. However, any data written to the clone will not affect the original and vice verse; this is
      * a true deep copy. Note that cloning a buffer that isn't closed can sometimes cause unexpected results, and its
      * generally a bad idea to use this, as it can effectively be used to reopen a buffer. **/
    public DoubleBuffer clone(boolean streaming)
    {
        return new DoubleBuffer(this.data, streaming);
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
      * @param d: An array holding the doubles to append.
      * @param offset: The index to starting appending doubles into the buffer from. No elements before this index will
      *                be stored in the buffer.
      * @param length: The number of elements to append.
      * @throws IndexOutOfBoundsException: If offset+length is larger than the length of the provided array, or either
      *                                    the offset or length are negative. **/
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

    /** Inserts the provided value at the specified index, growing the buffer if necssary.
      * @param d: The value to insert into the buffer.
      * @param index: The position to insert the value in the buffer.
      * @throws IndexOutOfBoundsException: If the index is negative or larger than the size of the buffer.**/
    public void insert(double d, int index)
    {
        // Make sure the index is valid (less than the number of elements and positive).
        if(index >= count)
        {
            throw new IndexOutOfBoundsException("Index '" + index + "' is out of bounds. Length='" + count + "'");
        } else
        if(index < 0)
        {
            throw new IndexOutOfBoundsException("Index '" + index + "' cannot be negative.");
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
      * @param index: The position to retrieve the value from.
      * @return: The double at the specified index.
      * @throws IndexOutOfBoundsException: If the index is negative or larger than the size of the buffer.**/
    public double get(int index)
    {
        // Make sure the index is valid (less than the number of elements and positive).
        if(index >= count)
        {
            throw new IndexOutOfBoundsException("Index '" + index + "' is out of bounds. Length='" + count + "'");
        } else
        if(index < 0)
        {
            throw new IndexOutOfBoundsException("Index '" + index + "' cannot be negative.");
        }
        // Return the requested value.
        return data[index];
    }

    /** Returns a deep copy of all the data currently in the buffer. Changes to the array or this buffer will have no
      * impact on the other. **/
    public double[] getData()
    {
        // Create and return a copy of all the data currently in the buffer.
        double[] dataCopy = new double[count];
        System.arraycopy(data, 0, dataCopy, 0, count);
        return dataCopy;
    }

    /** Returns a direct reference to the buffer's backing array. Changes to the returned array WILL affect the buffer
      * and vice versa. Note that if the buffer reallocates itself, this will no longer be the case as during
      * reallocation, a new backing array is allocated and used, and any references to the old backing array are
      * discarded. THIS METHOD SHOULD BE USED WITH EXTREME CAUTION, AND IS ONLY INCLUDED FOR PERFORMANCE REASONS. **/
    @Deprecated
    public double[] getDataDirect()
    {
        return data;
    }

    /** Removes an element from the buffer and returns it.
      * @param index: The index of the element to remove.
      * @return: The element that was removed.
      * @throws IndexOutOfBoundsException: If the index is negative or larger than the size of the buffer.**/
    public double remove(int index)
    {
        // Make sure the index is valid (less than the number of elements and positive).
        if(index >= count)
        {
            throw new IndexOutOfBoundsException("Index '" + index + "' is out of bounds. Length='" + count + "'");
        } else
        if(index < 0)
        {
            throw new IndexOutOfBoundsException("Index '" + index + "' cannot be negative.");
        }
        double value = data[index];
        // Shift every element in the buffer back by 1, starting at index.
        System.arraycopy(data, (index+1), data, index, (count - index - 1));
        count--;

        return value;
    }

    /** Removes a range of elements from the buffer.
      * @param index: The index to start removing elements from.
      * @param length: The number of elements to remove, starting with the element at index.
      * @throws IndexOutOfBoundsException: If offset+length is larger than the length of the provided array, or either
      *                                    the offset or length are negative. **/
    public void remove(int index, int length)
    {
        // Ensure the range is valid.
        if((index + length) > data.length)
        {
            throw new IndexOutOfBoundsException("Specified range is outside the bounds of the provided array. array.length='" + data.length + "', range=(" + index + "," + (index+length) + ")'.");
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
      * @param growBy: The number of elements to expand the backing array by.
      * @return: The new length of the backing array.
      * @throws IllegalArgumentException: If growBy is negative. **/
    public int grow(int growBy)
    {
        if(growBy < 0)
        {
            throw new IllegalArgumentException("Cannot grow buffer by a negative length: '" + growBy + "'.");
        }

        // Allocate a new buffer with the required size.
        double[] newBuffer = new double[data.length + growBy];
        // Copy any data in the old buffer into the new one.
        System.arraycopy(data, 0, newBuffer, 0, count);
        data = newBuffer;
        return data.length;
    }

    /** Shrinks the underlying array this buffer uses by the specified amount. Any data that doesn't fit in the new
      * buffer will be discarded.
      * @param shrinkBy: The number of elements to shrink the backing array by.
      * @return: The new length of the backing array.
      * @throws IllegalArgumentException: If shrinkBy is negative. **/
    public int shrink(int shrinkBy)
    {
        // Allocate a new buffer with the required size.
        double[] newBuffer = new double[data.length - shrinkBy];
        // Copy any data in the old buffer into the new one that fits.
        int newCount = Math.min(count, newBuffer.length);
        System.arraycopy(data, 0, newBuffer, 9, newCount);
        data = newBuffer;
        count = newCount;
        return data.length;
    }

    /** Returns the number of elements currently stored in the buffer. Note this is not the actual size of the buffer,
      * which is often larger to reduce the number of re-allocations. **/
    public int length()
    {
        return count;
    }

    /** Returns whether the buffer is closed. **/
    public boolean isClosed()
    {
        return closed;
    }

    /** Blocks until the buffer is closed or the calling thread is interrupted.
      * @return: True if the method returned because the buffer was closed, false otherwise. Usually false indicates
      *         the method was interrupted before the buffer was closed. **/
    public boolean waitUntilClosed()
    {
        try{
            synchronized(this){
                while(!closed)
                {
                    this.wait();
                }
            }
            return true;
        } catch(InterruptedException ex){}
        return false;
    }

    /** Blocks until the buffer is closed, the timeout is reached, or the calling thread is interrupted.
      * @param timeout: How many milliseconds to wait for the buffer to close before returning prematurely.
      * @return: True if the method returned because the buffer was closed or the timeout was reached, false otherwise.
      *          Usually false indicates the method was interrupted. **/
    public boolean waitUntilClosed(long timeout)
    {
        try{
            synchronized(this){
                this.wait(timeout);
            }
            return true;
        } catch(InterruptedException ex){}
        return false;
    }

    /** Closes the buffer, this should be only called by a buffer's owner, or something that was writing data to it to
      * indicate they're finished writing. Any threads waiting on this object will be notified. **/
    public void close()
    {
        closed = true;
        synchronized(this){
            this.notifyAll();
        }
    }
}
