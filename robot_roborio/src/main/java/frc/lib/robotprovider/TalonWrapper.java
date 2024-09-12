package frc.lib.robotprovider;

import edu.wpi.first.wpilibj.motorcontrol.Talon;

public class TalonWrapper implements IMotor
{
    private final Talon wrappedObject;

    public TalonWrapper(int channel)
    {
        this.wrappedObject = new Talon(channel);
    }

    public void set(double power)
    {
        this.wrappedObject.set(power);
    }
}
