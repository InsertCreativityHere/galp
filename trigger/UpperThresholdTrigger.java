
package net.insertcreativity.galp.trigger;

import net.insertcreativity.galp.Trigger;

/**
 * Trigger that activates when the value of a datapoint rises above a certain threshold value.
**/
public class UpperThresholdTrigger implements Trigger
{
    // The upper bound to check datapoints against.
    public final double upper;
    // True if the trigger should activate when the datapoint is exactly the upper bound, false if the datapoint must
    // be strictly greater than the upper bound.
    public final boolean equals;

    /** Creates a new threshold trigger with the specified upper bound. The trigger will activate when a data value
      * is greater than the upper bound.
      * @param upperBound: The upper bound; data values greater than this value will activate the trigger. **/
    public UpperThresholdTrigger(double upperBound)
    {
        this(upperBound, false);
    }

    /** Creates a new threshold trigger with the specified upper bound. The trigger will activate when a data value
      * is greater than the upper bound, or equal to it, if equalsTo is true.
      * @param upperBound: The upper bound; data values greater than this value will activate the trigger.
      * @param equalsTo: True if a data value being equal to upperBound should also activate the trigger. **/
    public UpperThresholdTrigger(double upperBound, boolean equalsTo)
    {
        upper = upperBound;
        equals = equalsTo;
    }

    /** Returns whether the trigger should be activated. This function should be called everytime a new datapoint
      * is received.
      * @param datapoint: The most recent datapoint value.
      * @return: Whether the datapoint is above the threshold value. **/
    public boolean checkTriggered(double datapoint)
    {
        return ((datapoint > upper) || (equals && (datapoint == upper)));
    }
}
