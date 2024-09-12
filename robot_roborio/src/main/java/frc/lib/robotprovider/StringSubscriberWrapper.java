package frc.lib.robotprovider;

import edu.wpi.first.networktables.StringSubscriber;

public class StringSubscriberWrapper implements IStringSubscriber
{
    final StringSubscriber wrappedObject;

    StringSubscriberWrapper(StringSubscriber object)
    {
        this.wrappedObject = object;
    }

    public String get()
    {
        return this.wrappedObject.get();
    }

    public String get(String defaultValue)
    {
        return this.wrappedObject.get(defaultValue);
    }
}