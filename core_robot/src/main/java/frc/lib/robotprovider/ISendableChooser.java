package frc.lib.robotprovider;

public interface ISendableChooser<V>
{
    void addDefault(String name, V object);

    void addObject(String name, V object);

    V getSelected();
}
