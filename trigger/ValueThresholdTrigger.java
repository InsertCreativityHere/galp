
package net.insertcreativity.galp.trigger;

import net.insertcreativity.galp.Trigger;

public class ValueThresholdTrigger implements Trigger
{
    public final double lower;
    public final double upper;
    public final boolean lowerEquals;
    public final boolean upperEquals;

    public ValueThresholdTrigger(double lowerBound, double upperBound)
    {
        this(lowerBound, upperBound, false);
    }

    public ValueThresholdTrigger(double lowerBound, double upperBound, boolean equalsTo)
    {
        this(lowerBound, upperBound, equalsTo, equalsTo);
    }

    public ValueThresholdTrigger(double lowerBound, double upperBound, boolean lowerEqualsTo, boolean upperEqualsTo)
    {
        lower = lowerBound;
        upper = upperBound;
        lowerEquals = lowerEqualsTo;
        upperEquals = upperEqualsTo;
    }

    public boolean isTriggered(double datapoint)
    {
        return ((datapoint > upper) || (datapoint < lower) || (upperEquals && (datapoint == upper)) || (lowerEquals && (datapoint == lower));
    }
}
