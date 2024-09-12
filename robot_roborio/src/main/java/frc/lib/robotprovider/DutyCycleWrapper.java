package frc.lib.robotprovider;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

public class DutyCycleWrapper implements IDutyCycle
{
    private final DutyCycle wrappedObject;

    public DutyCycleWrapper(int digitalInputChannel)
    {
        this.wrappedObject = new DutyCycle(new DigitalInput(digitalInputChannel));
    }

    public double getOutput()
    {
        return this.wrappedObject.getOutput();
    }

    public int getFrequency()
    {
        return this.wrappedObject.getFrequency();
    }
}
