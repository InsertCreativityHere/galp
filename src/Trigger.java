
package net.insertcreativity.galp;

/**
 * Base class all triggers implement. Triggers are used to evaluate certain conditions as data is received. These
 * can range from a simply activating a set period of time after data collection begins, to activating once the data
 * falls outside of a certain range. These are used to trigger events, like starting and stopping batch readings for
 * instance. Once a trigger has activated, it will always remain activated.
**/
public class Trigger
{
    // Whether the trigger has activated yet.
    private boolean activated;

    /** Returns whether the trigger has been activated yet. If it hasn't it checks to see if the trigger should be
      * activated and returns the result.
      * @param datapoint: The most recent datapoint collected by a sensor.
      * @return: Whether the trigger has been activated yet. **/
    public final boolean isTriggered(double datapoint)
    {
        if(!activated)
        {
            // Only check the trigger condition if the trigger hasn't already activated.
            activated = checkTriggered(datapoint);
        }
        return activated;
    }

    /** Check if the current datapoint value would cause the trigger to activate. **/
    public abstract boolean checkTriggered(double datapoint);

    /** Some triggers that are time-dependent shouldn't be started during construction, and so this method is provided
      * as an means of manually starting a trigger. **/
    public void start() {}
}
