package frc.lib.robotprovider;

import edu.wpi.first.networktables.DoubleSubscriber;

public class DoubleSubscriberWrapper implements IDoubleSubscriber
{
    final DoubleSubscriber wrappedObject;

    DoubleSubscriberWrapper(DoubleSubscriber object)
    {
        this.wrappedObject = object;
    }

    public double get()
    {
        return this.wrappedObject.get();
    }

    public double get(double defaultValue)
    {
        return this.wrappedObject.get(defaultValue);
    }
}