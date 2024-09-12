package frc.lib.robotprovider;

public class PneumaticHubWrapper
{
    static edu.wpi.first.wpilibj.PneumaticsModuleType getModuleType(PneumaticsModuleType moduleType)
    {
        if (moduleType == PneumaticsModuleType.PneumaticsControlModule)
        {
            return edu.wpi.first.wpilibj.PneumaticsModuleType.CTREPCM;
        }
        else // if (moduleType == PneumaticsModuleType.PneumaticsHub)
        {
            return edu.wpi.first.wpilibj.PneumaticsModuleType.REVPH;
        }
    }
}
