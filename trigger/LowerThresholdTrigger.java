
package net.insertcreativity.galp.trigger;

import net.insertcreativity.galp.Trigger;

public class LowerThresholdTrigger implements Trigger
{
    public final double lower;
    public final boolean equals;

    public LowerThresholdTrigger(double lowerBound)
    {
        this(lowerBound, false);
    }

    public LowerThresholdTrigger(double lowerBound, boolean equalsTo)
    {
        lower = lowerBound;
        equals = equalsTo;
    }

    public boolean isTriggered(double datapoint)
    {
        return ((datapoint < lower) || (equals && (datapoint == lower)));
    }
}
