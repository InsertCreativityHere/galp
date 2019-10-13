
package net.insertcreativity.galp.trigger;

import net.insertcreativity.galp.Trigger;

/**
 * Trigger that waits a specified amount of time before triggering.
**/
public class TimeDelayTrigger implements Trigger
{
    // The number of milliseconds to wait until triggering after being started.
    private final long delay;
    // The time that the trigger was started at.
    private long startTime = -1;
    // The time that the trigger should trigger at.
    private long endTime = LONG.MAX_VALUE;

    /** Creates a new TimeDelayTrigger with the specified delay and starts it immediately after construction.
        @param delay: How many milliseconds the trigger should wait after being started before triggering. **/
    public TimeDelayTrigger(long delay)
    {
        this(delay, true);
    }

    /** Creates a new TimeDelayTrigger with the specified delay.
        @param delay: How many milliseconds the trigger should wait after being started before triggering.
        @param startNow: Whether the trigger should be started immediately after creation. **/
    public TimeDelayTrigger(long delay, boolean startNow)
    {
        this.delay = delay;
        if(startNow)
        {
            start();
        }
    }

    /** Starts the TimeDelayTrigger, once started it'll wait the specified delay time until triggering. Trying to start
        a trigger that's already been started will throw an exception.
        @throws IllegalStateException: If start is called on a trigger that was already started. **/
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

    /** Returns true after the specified delay has passed after the trigger was started, and false until then. This
        will always return false for triggers that haven't been started yet. **/
    public boolean isTriggered(double datapoint)
    {
        return (System.currentTimeMillis >= endTime);
    }
}
