
package net.insertcreativity.galp.trigger;

import net.insertcreativity.galp.Trigger;

/**
 * Trigger that activates once a certain number of datapoint readings have been taken.
**/
public class DatapointCountTrigger extends Trigger
{
    // The maximum number of datapoint readings to take before activating. When this many datapoint readings have been
    // taken, the trigger will activate, this is because if any more readings were taken, it would go over maxCount.
    public final long maxCount;
    // The current number of datapoint readings taken since this trigger was created.
    public long count;

    /** Creates a new datapoint count trigger that activates once the number of datapoint readings taken is at least
      * maxCount.
      * @param maxCount: The maximum number of datapoint readings to take. Once this number of datapoint readings have
      *                  been taken, this trigger will activate. **/
    public DatapointCountTrigger(long maxCount)
    {
        this.maxCount = maxCount;
        count = 0;
    }

    /** Returns whether this trigger should be activated. This function should be called everytime a new datapoint
      * is received.
      * @param datapoint: The most recent datapoint.
      * @return True if after this method has completed, there will of been maxCount many datapoint readings taken. */
    public boolean checkTriggered(double datapoint)
    {
        count++;
        return (count > maxCount);
    }
}
