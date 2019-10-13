
package net.insertcreativity.galp.trigger;

import net.insertcreativity.galp.Trigger;

public class UpperThresholdTrigger implements Trigger
{
    public final double upper;
    public final boolean equals;

    public UpperThresholdTrigger(double uppderBound)
    {
        this(upperBound, false);
    }

    public UpperThresholdTrigger(double uppderBound, boolean equalsTo)
    {
        upper = upperBound;
        equals = equalsTo;
    }

    public boolean isTriggered(double datapoint)
    {
        return ((datapoint > upper) || (equals && (datapoint == upper)));
    }
}
