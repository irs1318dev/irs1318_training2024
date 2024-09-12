package frc.lib.robotprovider;

import edu.wpi.first.wpilibj.PowerDistribution;

public class PowerDistributionWrapper implements IPowerDistribution
{
    private final PowerDistribution wrappedObject;

    public PowerDistributionWrapper()
    {
        this.wrappedObject = new PowerDistribution();
    }

    public PowerDistributionWrapper(int module, PowerDistributionModuleType moduleType)
    {
        this.wrappedObject = new PowerDistribution(module, PowerDistributionWrapper.getModuleType(moduleType));
    }

    static edu.wpi.first.wpilibj.PowerDistribution.ModuleType getModuleType(PowerDistributionModuleType moduleType)
    {
        if (moduleType == PowerDistributionModuleType.PowerDistributionPanel)
        {
            return edu.wpi.first.wpilibj.PowerDistribution.ModuleType.kCTRE;
        }
        else // if (moduleType == PowerDistributionModuleType.PowerDistributionHub)
        {
            return edu.wpi.first.wpilibj.PowerDistribution.ModuleType.kRev;
        }
    }

    public double getBatteryVoltage()
    {
        return this.wrappedObject.getVoltage();
    }

    public double getCurrent(int pdpChannel)
    {
        return this.wrappedObject.getCurrent(pdpChannel);
    }

    public double getTotalCurrent()
    {
        return this.wrappedObject.getTotalCurrent();
    }

    public double getTotalEnergy()
    {
        return this.wrappedObject.getTotalEnergy();
    }

    public double getTotalPower()
    {
        return this.wrappedObject.getTotalPower();
    }

    public double getTemperature()
    {
        return this.wrappedObject.getTemperature();
    }

    public void setSwitchableChannel(boolean enabled)
    {
        this.wrappedObject.setSwitchableChannel(enabled);
    }
}
