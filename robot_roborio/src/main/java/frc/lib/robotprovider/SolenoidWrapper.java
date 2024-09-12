package frc.lib.robotprovider;

import edu.wpi.first.wpilibj.Solenoid;

public class SolenoidWrapper implements ISolenoid
{
    private final Solenoid wrappedObject;

    public SolenoidWrapper(PneumaticsModuleType moduleType, int channel)
    {
        this.wrappedObject = new Solenoid(PneumaticHubWrapper.getModuleType(moduleType), channel);
    }

    public SolenoidWrapper(int moduleNumber, PneumaticsModuleType moduleType, int channel)
    {
        this.wrappedObject = new Solenoid(moduleNumber, PneumaticHubWrapper.getModuleType(moduleType), channel);
    }

    public void set(boolean on)
    {
        this.wrappedObject.set(on);
    }
}
