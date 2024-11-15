package frc.lib.robotprovider;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;

public class CANCoderWrapper implements ICANCoder
{
    private static final double timeoutSecs = 0.025;

    private final CANcoder wrappedObject;

    private final String cancoderId;

    private boolean reverse;

    private StatusSignal<Double> position;
    private StatusSignal<Double> absolutePosition;
    private StatusSignal<Double> velocity;

    public CANCoderWrapper(int deviceNumber)
    {
        this.wrappedObject = new CANcoder(deviceNumber);
        this.cancoderId = String.format("CANcoder %d", deviceNumber);
        this.reverse = false;
    }

    public CANCoderWrapper(int deviceNumber, String canbus)
    {
        this.wrappedObject = new CANcoder(deviceNumber, canbus);
        this.cancoderId = String.format("CANcoder %s-%d", canbus, deviceNumber);
        this.reverse = false;
    }

    public double getPosition()
    {
        if (this.position == null)
        {
            this.position = this.wrappedObject.getPosition();
        }

        this.position.refresh();
        CTREStatusCodeHelper.printError(this.position.getStatus(), this.cancoderId, "CANCoderWrapper.getPosition");
        return reverse ? -this.position.getValue() : this.position.getValue();
    }

    public double getVelocity()
    {
        if (this.velocity == null)
        {
            this.velocity = this.wrappedObject.getVelocity();
        }

        this.velocity.refresh();
        CTREStatusCodeHelper.printError(this.velocity.getStatus(), this.cancoderId, "CANCoderWrapper.getVelocity");
        return reverse ? -this.velocity.getValue() : this.velocity.getValue();
    }

    public double getAbsolutePosition()
    {
        if (this.absolutePosition == null)
        {
            this.absolutePosition = this.wrappedObject.getAbsolutePosition();
        }

        this.absolutePosition.refresh();
        CTREStatusCodeHelper.printError(this.absolutePosition.getStatus(), this.cancoderId, "CANCoderWrapper.getAbsolutePosition");
        return reverse ? -this.absolutePosition.getValue() : this.absolutePosition.getValue();
    }

    public void setPosition(double newPosition)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.setPosition(newPosition, CANCoderWrapper.timeoutSecs),
            this.cancoderId,
            "CANCoderWrapper.setPosition");
    }

    public void configSensorDirection(boolean clockwisePositive)
    {
        this.reverse = clockwisePositive;
    }
}