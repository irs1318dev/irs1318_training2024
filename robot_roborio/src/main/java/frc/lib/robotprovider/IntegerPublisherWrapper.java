package frc.lib.robotprovider;

import edu.wpi.first.networktables.IntegerPublisher;

public class IntegerPublisherWrapper implements IIntegerPublisher
{
    final IntegerPublisher wrappedObject;

    IntegerPublisherWrapper(IntegerPublisher object)
    {
        this.wrappedObject = object;
    }

    public void set(long value)
    {
        this.wrappedObject.set(value);
    }

    public void setDefault(long defaultValue)
    {
        this.wrappedObject.setDefault(defaultValue);
    }
}