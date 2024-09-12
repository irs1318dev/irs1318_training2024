package frc.lib.robotprovider;

import edu.wpi.first.networktables.IntegerSubscriber;

public class IntegerSubscriberWrapper implements IIntegerSubscriber
{
    final IntegerSubscriber wrappedObject;

    IntegerSubscriberWrapper(IntegerSubscriber object)
    {
        this.wrappedObject = object;
    }

    public long get()
    {
        return this.wrappedObject.get();
    }

    public long get(long defaultValue)
    {
        return this.wrappedObject.get(defaultValue);
    }
}