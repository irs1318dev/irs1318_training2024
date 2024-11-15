package frc.lib.robotprovider;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import frc.lib.helpers.ExceptionHelpers;

public class TalonSRXWrapper implements ITalonSRX
{
    private static final int pidIdx = 0;
    private static final int timeoutMS = 10;

    final TalonSRX wrappedObject;

    private final String talonId;

    private boolean controlModeRequired;
    private ControlMode controlMode;

    public TalonSRXWrapper(int deviceNumber)
    {
        this.wrappedObject = new TalonSRX(deviceNumber);
        this.talonId = String.format("TalonSRX %d", deviceNumber);
        this.controlMode = ControlMode.PercentOutput;
        this.controlModeRequired = false;
    }

    public void set(double value)
    {
        ExceptionHelpers.Assert(!this.controlModeRequired, "%s: Control mode must be specified!", this.talonId);

        this.wrappedObject.set(this.controlMode, value);
    }

    public void set(double value, double feedForward)
    {
        ExceptionHelpers.Assert(!this.controlModeRequired, "%s: Control mode must be specified!", this.talonId);

        this.wrappedObject.set(this.controlMode, value, DemandType.ArbitraryFeedForward, feedForward);
    }

    public void set(TalonSRXControlMode mode, double value)
    {
        this.wrappedObject.set(TalonSRXWrapper.getControlMode(mode), value);
    }

    public void set(TalonSRXControlMode mode, double value, double feedForward)
    {
        this.wrappedObject.set(TalonSRXWrapper.getControlMode(mode), value, DemandType.ArbitraryFeedForward, feedForward);
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
        this.controlModeRequired = (mode == TalonSRXControlMode.Required);
        this.controlMode = TalonSRXWrapper.getControlMode(mode);
    }

    public void setSensorType(TalonSRXFeedbackDevice feedbackDevice)
    {
        FeedbackDevice device;
        if (feedbackDevice == TalonSRXFeedbackDevice.QuadEncoder)
        {
            device = FeedbackDevice.QuadEncoder;
        }
        else if (feedbackDevice == TalonSRXFeedbackDevice.PulseWidthEncodedPosition)
        {
            device = FeedbackDevice.PulseWidthEncodedPosition;
        }
        else
        {
            return;
        }

        CTREStatusCodeHelper.printError(
            this.wrappedObject.configSelectedFeedbackSensor(device, TalonSRXWrapper.pidIdx, TalonSRXWrapper.timeoutMS),
            this.talonId,
            "TalonSRX.setSensorType");
    }

    public void setGeneralFramePeriod(int periodMS)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, periodMS, TalonSRXWrapper.timeoutMS),
            this.talonId,
            "TalonSRX.setGeneralFramePeriod");
    }

    public void setFeedbackFramePeriod(int periodMS)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, periodMS, TalonSRXWrapper.timeoutMS),
            this.talonId,
            "TalonSRX.setFeedbackFramePeriod");
    }

    public void setPIDFFramePeriod(int periodMS)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, periodMS, TalonSRXWrapper.timeoutMS),
            this.talonId,
            "TalonSRX.setPIDFFramePeriod");
    }

    public void configureVelocityMeasurements(int periodMS, int windowSize)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.valueOf(periodMS), TalonSRXWrapper.timeoutMS),
            this.talonId,
            "TalonSRX.configureVelocityMeasurementsPeriod");
        CTREStatusCodeHelper.printError(
            this.wrappedObject.configVelocityMeasurementWindow(windowSize, TalonSRXWrapper.timeoutMS),
            this.talonId,
            "TalonSRX.configureVelocityMeasurementsWindow");
    }

    public void configureAllowableClosedloopError(int slotId, int error)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.configAllowableClosedloopError(slotId, error, TalonSRXWrapper.timeoutMS),
            this.talonId,
            "TalonSRX.configureAllowableClosedloopError");
    }

    public void setSelectedSlot(int slotId)
    {
        this.wrappedObject.selectProfileSlot(slotId, TalonSRXWrapper.pidIdx);
    }

    public void setPIDF(double p, double i, double d, double f, int slotId)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.config_kP(slotId, p, TalonSRXWrapper.timeoutMS),
            this.talonId,
            "TalonSRX.setPIDF_kP");
        CTREStatusCodeHelper.printError(
            this.wrappedObject.config_kI(slotId, i, TalonSRXWrapper.timeoutMS),
            this.talonId,
            "TalonSRX.setPIDF_kI");
        CTREStatusCodeHelper.printError(
            this.wrappedObject.config_kD(slotId, d, TalonSRXWrapper.timeoutMS),
            this.talonId,
            "TalonSRX.setPIDF_kD");
        CTREStatusCodeHelper.printError(
            this.wrappedObject.config_kF(slotId, f, TalonSRXWrapper.timeoutMS),
            this.talonId,
            "TalonSRX.setPIDF_kF");
    }

    public void setMotionMagicPIDF(double p, double i, double d, double f, double velocity, double acceleration, int slotId)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.config_kP(slotId, p, TalonSRXWrapper.timeoutMS),
            this.talonId,
            "TalonSRX.setMotionMagicPIDF_kP");
        CTREStatusCodeHelper.printError(
            this.wrappedObject.config_kI(slotId, i, TalonSRXWrapper.timeoutMS),
            this.talonId,
            "TalonSRX.setMotionMagicPIDF_kI");
        CTREStatusCodeHelper.printError(
            this.wrappedObject.config_kD(slotId, d, TalonSRXWrapper.timeoutMS),
            this.talonId,
            "TalonSRX.setMotionMagicPIDF_kD");
        CTREStatusCodeHelper.printError(
            this.wrappedObject.config_kF(slotId, f, TalonSRXWrapper.timeoutMS),
            this.talonId,
            "TalonSRX.setMotionMagicPIDF_kF");
        CTREStatusCodeHelper.printError(
            this.wrappedObject.configMotionCruiseVelocity(velocity, TalonSRXWrapper.timeoutMS),
            this.talonId,
            "TalonSRX.setMotionMagicPIDF_CruiseVelocity");
        CTREStatusCodeHelper.printError(
            this.wrappedObject.configMotionAcceleration(acceleration, TalonSRXWrapper.timeoutMS),
            this.talonId,
            "TalonSRX.setMotionMagicPIDF_Acceleration");
    }

    public void setPIDF(double p, double i, double d, double f, int izone, double closeLoopRampRate, int slotId)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.config_kP(slotId, p, TalonSRXWrapper.timeoutMS),
            this.talonId,
            "TalonSRX.setPIDF_kP");
        CTREStatusCodeHelper.printError(
            this.wrappedObject.config_kI(slotId, i, TalonSRXWrapper.timeoutMS),
            this.talonId,
            "TalonSRX.setPIDF_kI");
        CTREStatusCodeHelper.printError(
            this.wrappedObject.config_kD(slotId, d, TalonSRXWrapper.timeoutMS),
            this.talonId,
            "TalonSRX.setPIDF_kD");
        CTREStatusCodeHelper.printError(
            this.wrappedObject.config_kF(slotId, f, TalonSRXWrapper.timeoutMS),
            this.talonId,
            "TalonSRX.setPIDF_kF");
        CTREStatusCodeHelper.printError(
            this.wrappedObject.config_IntegralZone(slotId, izone, TalonSRXWrapper.timeoutMS),
            this.talonId,
            "TalonSRX.setPIDF_IntegralZone");
        CTREStatusCodeHelper.printError(
            this.wrappedObject.configClosedloopRamp(closeLoopRampRate, TalonSRXWrapper.timeoutMS),
            this.talonId,
            "TalonSRX.setPIDF_ClosedloopRamp");
    }

    public void updateLimitSwitchConfig(boolean forwardEnabled, boolean forwardNormallyOpen, boolean reverseEnabled, boolean reverseNormallyOpen)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.configForwardLimitSwitchSource(
                forwardEnabled ? LimitSwitchSource.FeedbackConnector : LimitSwitchSource.Deactivated,
                forwardNormallyOpen ? LimitSwitchNormal.NormallyOpen : LimitSwitchNormal.NormallyClosed,
                TalonSRXWrapper.timeoutMS),
                this.talonId,
            "TalonSRX.updateLimitSwitchConfig-forward");

        CTREStatusCodeHelper.printError(
            this.wrappedObject.configReverseLimitSwitchSource(
                reverseEnabled ? LimitSwitchSource.FeedbackConnector : LimitSwitchSource.Deactivated,
                reverseNormallyOpen ? LimitSwitchNormal.NormallyOpen : LimitSwitchNormal.NormallyClosed,
                TalonSRXWrapper.timeoutMS),
                this.talonId,
            "TalonSRX.updateLimitSwitchConfig-reverse");
    }

    public void setMotorOutputSettings(boolean invert, MotorNeutralMode neutralMode)
    {
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
        this.wrappedObject.setInverted(invert);
    }

    public void setInvertSensor(boolean invert)
    {
        this.wrappedObject.setSensorPhase(invert);
    }

    public void setVoltageCompensation(boolean enabled, double maxVoltage)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.configVoltageCompSaturation(maxVoltage, TalonSRXWrapper.timeoutMS),
            this.talonId,
            "TalonSRX.setVoltageCompensationSaturation");
        this.wrappedObject.enableVoltageCompensation(enabled);
    }

    public void stop()
    {
        this.wrappedObject.set(ControlMode.Disabled, 0.0);
    }

    public void setPosition(double position)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.setSelectedSensorPosition(position, TalonSRXWrapper.pidIdx, TalonSRXWrapper.timeoutMS),
            this.talonId,
            "TalonSRX.setPosition");
    }

    public void reset()
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.setSelectedSensorPosition(0.0, TalonSRXWrapper.pidIdx, TalonSRXWrapper.timeoutMS),
            this.talonId,
            "TalonSRX.reset");
    }

    public double getPosition()
    {
        return this.wrappedObject.getSelectedSensorPosition(TalonSRXWrapper.pidIdx);
    }

    public double getVelocity()
    {
        return this.wrappedObject.getSelectedSensorVelocity(TalonSRXWrapper.pidIdx);
    }

    public double getError()
    {
        return this.wrappedObject.getClosedLoopError(TalonSRXWrapper.pidIdx);
    }

    public double getOutput()
    {
        return this.wrappedObject.getMotorOutputPercent();
    }

    public TalonXLimitSwitchStatus getLimitSwitchStatus()
    {
        SensorCollection collection = this.wrappedObject.getSensorCollection();

        return new TalonXLimitSwitchStatus(
            collection.isFwdLimitSwitchClosed(),
            collection.isRevLimitSwitchClosed());
    }

    static ControlMode getControlMode(TalonSRXControlMode mode)
    {
        if (mode == TalonSRXControlMode.PercentOutput)
        {
            return ControlMode.PercentOutput;
        }
        else if (mode == TalonSRXControlMode.Disabled)
        {
            return ControlMode.Disabled;
        }
        else if (mode == TalonSRXControlMode.Follower)
        {
            return ControlMode.Follower;
        }
        else if (mode == TalonSRXControlMode.Position)
        {
            return ControlMode.Position;
        }
        else if (mode == TalonSRXControlMode.MotionMagicPosition)
        {
            return ControlMode.MotionMagic;
        }
        else if (mode == TalonSRXControlMode.Velocity)
        {
            return ControlMode.Velocity;
        }
        else if (mode == TalonSRXControlMode.Current)
        {
            return ControlMode.Current;
        }

        return ControlMode.PercentOutput;
    }
}
