package frc.lib.robotprovider;

import frc.lib.helpers.ExceptionHelpers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class VictorSPXWrapper implements IVictorSPX
{
    final VictorSPX wrappedObject;

    private ControlMode controlMode;

    public VictorSPXWrapper(int deviceNumber)
    {
        this.wrappedObject = new VictorSPX(deviceNumber);
        this.controlMode = ControlMode.PercentOutput;
    }

    public void set(double value)
    {
        this.wrappedObject.set(this.controlMode, value);
    }

    public void follow(ITalonSRX talonSRX)
    {
        this.wrappedObject.follow(((TalonSRXWrapper)talonSRX).wrappedObject);
    }

    public void follow(IVictorSPX victorSPX)
    {
        this.wrappedObject.follow(((VictorSPXWrapper)victorSPX).wrappedObject);
    }

    public void setControlMode(TalonSRXControlMode mode)
    {
        if (mode == TalonSRXControlMode.PercentOutput)
        {
            this.controlMode = ControlMode.PercentOutput;
        }
        else if (mode == TalonSRXControlMode.Disabled)
        {
            this.controlMode = ControlMode.Disabled;
        }
        else if (mode == TalonSRXControlMode.Follower)
        {
            this.controlMode = ControlMode.Follower;
        }
        else
        {
            ExceptionHelpers.Assert(false, "don't allow other control modes " + mode);
        }
    }

    public void setMotorOutputSettings(boolean invert, MotorNeutralMode neutralMode)
    {
        this.wrappedObject.setInverted(invert);

        NeutralMode mode;
        if (neutralMode == MotorNeutralMode.Brake)
        {
            mode = NeutralMode.Brake;
        }
        else
        {
            mode = NeutralMode.Coast;
        }

        this.wrappedObject.setNeutralMode(mode);
    }

    public void stop()
    {
        this.wrappedObject.set(ControlMode.Disabled, 0.0);
    }
}
