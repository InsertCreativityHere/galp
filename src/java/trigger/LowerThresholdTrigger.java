
package net.insertcreativity.galp.trigger;

import net.insertcreativity.galp.Trigger;

/**
 * Trigger that activates when the value of a datapoint drops below a certain threshold value.
**/
public class LowerThresholdTrigger extends Trigger
{
    // The lower bound to check datapoints against.
    public final double lower;
    // True if the trigger should activate when the datapoint is exactly the lower bound, false if the datapoint must
    // be strictly less than the lower bound.
    public final boolean equals;

    /** Creates a new threshold trigger with the specified lower bound. The trigger will activate when a data value
      * is less than the lower bound.
      * @param lowerBound: The lower bound; data values less than this value will activate the trigger. **/
    public LowerThresholdTrigger(double lowerBound)
    {
        this(lowerBound, false);
    }

    /** Creates a new threshold trigger with the specified lower bound. The trigger will activate when a data value
      * is less than the lower bound, or equal to it, if equalsTo is true.
      * @param lowerBound: The lower bound; data values less than this value will activate the trigger.
      * @param equalsTo: True if a data value being equal to lowerBound should also activate the trigger. */
    public LowerThresholdTrigger(double lowerBound, boolean equalsTo)
    {
        lower = lowerBound;
        equals = equalsTo;
    }

    /** Returns whether the trigger should be activated. This function should be called everytime a new datapoint
      * is received.
      * @param datapoint: The most recent datapoint value.
      * @return: Whether the datapoint is below the threshold value. **/
    @Override
    public boolean checkTriggered(double datapoint)
    {
        return ((datapoint < lower) || (equals && (datapoint == lower)));
    }
}
