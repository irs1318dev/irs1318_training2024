package frc.lib.robotprovider;

import edu.wpi.first.networktables.DoublePublisher;

public class DoublePublisherWrapper implements IDoublePublisher
{
    final DoublePublisher wrappedObject;

    DoublePublisherWrapper(DoublePublisher object)
    {
        this.wrappedObject = object;
    }

    public void set(double value)
    {
        this.wrappedObject.set(value);
    }

    public void setDefault(double defaultValue)
    {
        this.wrappedObject.setDefault(defaultValue);
    }
}