
package net.insertcreativity.galp.trigger;

import net.insertcreativity.galp.Trigger;

/**
 * Trigger that activates once a datapoint is no longer within a specified range.
**/
public class ValueThresholdTrigger extends Trigger
{
    // The lower bound of the range.
    public final double lower;
    // The upper bound of the range.
    public final double upper;
    // Whether the range includes the lower bound.
    public final boolean lowerEquals;
    // Whether the range include the upper bound.
    public final boolean upperEquals;

    /** Creates a new threshold trigger that activates once a datapoint is no longer between lowerBound and upperBound.
      * The bounds are counted as being in the range; it's a closed interval.
      * @param lowerBound: The lower bound; data values less to this will activate the trigger.
      * @param upperBound: The upper bound; data values greater than this will activate the trigger. **/
    public ValueThresholdTrigger(double lowerBound, double upperBound)
    {
        this(lowerBound, upperBound, true);
    }

    /** Creates a new threshold trigger that activates once a datapoint is no longer between lowerBound and upperBound.
      * If equalsTo is true, the bounds are counted as part of the range (it's a closed interval), otherwise the bounds
      * aren't counted as part of the range (it's an open interval).
      * @param lowerBound: The lower bound; data values less than this will activate the trigger.
      * @param upperBound: The upper bound; data values greater than this will activate the trigger.
      * @param equalsTo: Whether the bounds should be counted as part of the interval. If this is false, then even data
      *                  values that are equal to the bounds will activate the trigger. **/
    public ValueThresholdTrigger(double lowerBound, double upperBound, boolean equalsTo)
    {
        this(lowerBound, upperBound, equalsTo, equalsTo);
    }

    /** Creates a new threshold trigger that activates once a datapoint is no longer between lowerBound and upperBound.
      * Whether each bound counts as part of the interval can be controlled with lowerEqualsTo and upperEqualsTo.
      * @param lowerBound: The lower bound; data values less than this will activate the trigger.
      * @param upperBound: The upper bound; data values greater than this will activate the trigger.
      * @param lowerEqualsTo: Whether the lower bound should be counted as part of the interval.
      * @param upperEqualsTo: Whether the upper bound should be counted as part of the interval. **/
    public ValueThresholdTrigger(double lowerBound, double upperBound, boolean lowerEqualsTo, boolean upperEqualsTo)
    {
        lower = lowerBound;
        upper = upperBound;
        lowerEquals = lowerEqualsTo;
        upperEquals = upperEqualsTo;
    }

    /** Returns whether the trigger should be activated. This function should be called everytime a new datapoint
      * is received.
      * @param datapoint: The most recent datapoint value.
      * @return: True if the datapoint is outside the range of this threshold trigger. False if it's within the
      *          range. **/
    public boolean checkTriggered(double datapoint)
    {
        return ((datapoint > upper) || (datapoint < lower) || (upperEquals && (datapoint == upper)) || (lowerEquals && (datapoint == lower)));
    }
}
