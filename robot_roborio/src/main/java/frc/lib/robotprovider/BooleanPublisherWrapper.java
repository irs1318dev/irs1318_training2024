package frc.lib.robotprovider;

import edu.wpi.first.networktables.BooleanPublisher;

public class BooleanPublisherWrapper implements IBooleanPublisher
{
    final BooleanPublisher wrappedObject;

    BooleanPublisherWrapper(BooleanPublisher object)
    {
        this.wrappedObject = object;
    }

    public void set(boolean value)
    {
        this.wrappedObject.set(value);
    }

    public void setDefault(boolean defaultValue)
    {
        this.wrappedObject.setDefault(defaultValue);
    }
}