package frc.lib.robotprovider;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AKSendableChooserWrapper<V> implements ISendableChooser<V>
{
    final LoggedDashboardChooser<V> wrappedObject;

    AKSendableChooserWrapper(String key)
    {
        this.wrappedObject = new LoggedDashboardChooser<V>(key);
    }

    @Override
    public void addDefault(String name, V object)
    {
        this.wrappedObject.addDefaultOption(name, object);
    }

    @Override
    public void addObject(String name, V object)
    {
        this.wrappedObject.addOption(name, object);
    }

    @Override
    public V getSelected()
    {
        return this.wrappedObject.get();
    }
}
