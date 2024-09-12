package frc.lib.robotprovider;

import edu.wpi.first.networktables.DoubleEntry;

public class DoubleEntryWrapper implements IDoubleEntry
{
    final DoubleEntry wrappedObject;

    DoubleEntryWrapper(DoubleEntry object)
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

    public void set(double value)
    {
        this.wrappedObject.set(value);
    }

    public void setDefault(double defaultValue)
    {
        this.wrappedObject.setDefault(defaultValue);
    }
}