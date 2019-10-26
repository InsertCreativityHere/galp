
package net.insertcreativity.galp.trigger;

import net.insertcreativity.galp.Trigger;

/**
 * Trigger that waits a specified amount of time before activating.
**/
public class TimeDelayTrigger extends Trigger
{
    // The number of milliseconds to wait until activating after being started.
    private final long delay;
    // The time that the trigger was started at.
    private long startTime = -1;
    // The time that the trigger should activate at.
    private long endTime = Long.MAX_VALUE;

    /** Creates a new TimeDelayTrigger with the specified delay and immediately starts it after construction.
      * @param delay: How many milliseconds the trigger should wait after being started before activating. **/
    public TimeDelayTrigger(long delay)
    {
        this(delay, true);
    }

    /** Creates a new TimeDelayTrigger with the specified delay.
      * @param delay: How many milliseconds the trigger should wait after being started before activating.
      * @param startNow: Whether the trigger should be started immediately after creation. **/
    public TimeDelayTrigger(long delay, boolean startNow)
    {
        this.delay = delay;
        if(startNow)
        {
            start();
        }
    }

    /** Starts the TimeDelayTrigger, once started it'll wait for the specified delay before activating. Trying to start
      * a trigger that's already been started will throw an exception.
      * @throws IllegalStateException: If start is called on a trigger that was already started. **/
    public void start()
    {
        if(startTime != -1)
        {
            startTime = System.currentTimeMillis();
            endTime = startTime + delay;
        } else {
            throw new IllegalStateException("Cannot restart a trigger once it's already been started.");
        }
    }

    /** Returns whether the trigger should be activated. This function should be called everytime a new datapoint
      * is received.
      * @param datapoint: The most recent datapoint value.
      * @return: Whether delay many seconds have passed since this trigger was started. **/
    @Override
    public boolean checkTriggered(double datapoint)
    {
        return (System.currentTimeMillis() >= endTime);
    }
}
