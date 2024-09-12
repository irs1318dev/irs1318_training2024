package frc.lib.robotprovider;

import edu.wpi.first.networktables.StringEntry;

public class StringEntryWrapper implements IStringEntry
{
    final StringEntry wrappedObject;

    StringEntryWrapper(StringEntry object)
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

    public void set(String value)
    {
        this.wrappedObject.set(value);
    }

    public void setDefault(String defaultValue)
    {
        this.wrappedObject.setDefault(defaultValue);
    }
}