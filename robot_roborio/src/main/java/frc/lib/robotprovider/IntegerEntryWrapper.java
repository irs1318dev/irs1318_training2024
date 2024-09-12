package frc.lib.robotprovider;

import edu.wpi.first.networktables.IntegerEntry;

public class IntegerEntryWrapper implements IIntegerEntry
{
    final IntegerEntry wrappedObject;

    IntegerEntryWrapper(IntegerEntry object)
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

    public void set(long value)
    {
        this.wrappedObject.set(value);
    }

    public void setDefault(long defaultValue)
    {
        this.wrappedObject.setDefault(defaultValue);
    }
}