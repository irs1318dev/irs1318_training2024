package frc.lib.robotprovider;

import edu.wpi.first.networktables.StringPublisher;

public class StringPublisherWrapper implements IStringPublisher
{
    final StringPublisher wrappedObject;

    StringPublisherWrapper(StringPublisher object)
    {
        this.wrappedObject = object;
    }

    public void set(String value)
    {
        this.wrappedObject.set(value);
    }

    public void setDefault(String defaultValue)
    {
        this.wrappedObject.setDefault(defaultValue);
    }
}