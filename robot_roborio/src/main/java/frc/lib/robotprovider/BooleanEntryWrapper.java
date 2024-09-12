package frc.lib.robotprovider;

import edu.wpi.first.networktables.BooleanEntry;

public class BooleanEntryWrapper implements IBooleanEntry
{
    final BooleanEntry wrappedObject;

    BooleanEntryWrapper(BooleanEntry object)
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

    public void set(boolean value)
    {
        this.wrappedObject.set(value);
    }

    public void setDefault(boolean defaultValue)
    {
        this.wrappedObject.setDefault(defaultValue);
    }
}