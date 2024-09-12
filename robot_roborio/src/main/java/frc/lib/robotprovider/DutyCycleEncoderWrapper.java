package frc.lib.robotprovider;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class DutyCycleEncoderWrapper implements IDutyCycleEncoder
{
    private final DutyCycleEncoder wrappedObject;

    public DutyCycleEncoderWrapper(int digitalInputChannel)
    {
        this.wrappedObject = new DutyCycleEncoder(digitalInputChannel);
    }

    public double get()
    {
        return this.wrappedObject.get();
    }

    public double getDistance()
    {
        return this.wrappedObject.getDistance();
    }

    public double getAbsolutePosition()
    {
        return this.wrappedObject.getAbsolutePosition();
    }

    public int getFrequency()
    {
        return this.wrappedObject.getFrequency();
    }

    public boolean isConnected()
    {
        return this.wrappedObject.isConnected();
    }

    public void setConnectedFrequencyThreshold(int frequency)
    {
        this.wrappedObject.setConnectedFrequencyThreshold(frequency);
    }

    public void setDistancePerRotation(double distancePerRotation)
    {
        this.wrappedObject.setDistancePerRotation(distancePerRotation);
    }

    public void setDutyCycleRange(double min, double max)
    {
        this.wrappedObject.setDutyCycleRange(min, max);
    }

    public void setPositionOffset(double offset)
    {
        this.wrappedObject.setPositionOffset(offset);
    }

    public void reset()
    {
        this.wrappedObject.reset();
    }
}
