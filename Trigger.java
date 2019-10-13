
package net.insertcreativity.galp;

/**
 * Base interface all triggers implement. Triggers are used to 'trigger' events in the sensor and sensor interfaces.
 * Most often events like starting and stopping a batch reading. Triggers are usually polled in a loop at the behest
 * of the object checking it. A Trigger is considered 'triggered' when 'isTriggered' returns true.
**/
public interface Trigger
{
    /** Returns whether or not the trigger is triggered.
        @param datapoint: The most recent datapoint collected by a sensor.
        @return: True if the trigger was triggered. **/
    public boolean isTriggered(double datapoint);

    /** Some triggers that are time-dependent shouldn't be started during construction, and so this method is provided
        as an means of manually starting a trigger. **/
    public default void start() {}
}
