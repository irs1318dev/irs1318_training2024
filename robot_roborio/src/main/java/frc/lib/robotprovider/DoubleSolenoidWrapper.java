package frc.lib.robotprovider;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class DoubleSolenoidWrapper implements IDoubleSolenoid
{
    private final DoubleSolenoid wrappedObject;

    public DoubleSolenoidWrapper(PneumaticsModuleType moduleType, int forwardChannel, int reverseChannel)
    {
        this.wrappedObject = new DoubleSolenoid(PneumaticHubWrapper.getModuleType(moduleType), forwardChannel, reverseChannel);
    }

    public DoubleSolenoidWrapper(int moduleNumber, PneumaticsModuleType moduleType, int forwardChannel, int reverseChannel)
    {
        this.wrappedObject = new DoubleSolenoid(moduleNumber, PneumaticHubWrapper.getModuleType(moduleType), forwardChannel, reverseChannel);
    }

    public void set(DoubleSolenoidValue value)
    {
        Value wpilibValue = Value.kOff;
        if (value == DoubleSolenoidValue.Forward)
        {
            wpilibValue = Value.kForward;
        }
        else if (value == DoubleSolenoidValue.Reverse)
        {
            wpilibValue = Value.kReverse;
        }

        this.wrappedObject.set(wpilibValue);
    }
}
