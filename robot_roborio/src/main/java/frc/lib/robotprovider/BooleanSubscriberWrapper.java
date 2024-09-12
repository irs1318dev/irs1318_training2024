package frc.lib.robotprovider;

import edu.wpi.first.networktables.BooleanSubscriber;

public class BooleanSubscriberWrapper implements IBooleanSubscriber
{
    final BooleanSubscriber wrappedObject;

    BooleanSubscriberWrapper(BooleanSubscriber object)
    {
        this.wrappedObject = object;
    }

    public boolean get()
    {
        return this.wrappedObject.get();
    }

    public boolean get(boolean defaultValue)
    {
        return this.wrappedObject.get(defaultValue);
    }
}