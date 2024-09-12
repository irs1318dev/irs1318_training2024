package frc.lib.robotprovider;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.SparkPIDController.AccelStrategy;
import com.revrobotics.SparkPIDController.ArbFFUnits;

public class SparkMaxWrapper implements ISparkMax
{
    final CANSparkMax wrappedObject;
    private SparkPIDController pidController;
    private boolean useAbsoluteEncoder;
    private RelativeEncoder wrappedRelativeEncoder;
    private AbsoluteEncoder wrappedAbsoluteEncoder;
    private SparkLimitSwitch wrappedFwdLimitSwitch;
    private SparkLimitSwitch wrappedRevLimitSwitch;

    private SparkMaxControlMode currentMode;
    private int selectedSlot;

    public SparkMaxWrapper(int deviceID, SparkMaxMotorType motorType)
    {
        CANSparkLowLevel.MotorType type = CANSparkLowLevel.MotorType.kBrushless;
        switch (motorType)
        {
            case Brushed:
                type = CANSparkLowLevel.MotorType.kBrushed;
                break;

            case Brushless:
                type = CANSparkLowLevel.MotorType.kBrushless;
                break;
        }

        this.wrappedObject = new CANSparkMax(deviceID, type);
        this.currentMode = SparkMaxControlMode.PercentOutput;
        this.useAbsoluteEncoder = false;
    }

    public void setControlMode(SparkMaxControlMode mode)
    {
        this.currentMode = mode;
    }

    public void setAbsoluteEncoder()
    {
        this.useAbsoluteEncoder = true;
        this.wrappedAbsoluteEncoder = this.wrappedObject.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    }

    public void setRelativeEncoder()
    {
        this.setRelativeEncoder(SparkMaxRelativeEncoderType.HallEffect, 42);
    }

    public void setRelativeEncoder(SparkMaxRelativeEncoderType encoderType, int resolution)
    {
        SparkRelativeEncoder.Type type;
        switch (encoderType)
        {
            case HallEffect:
                type = SparkRelativeEncoder.Type.kHallSensor;
                break;

            case Quadrature:
                type = SparkRelativeEncoder.Type.kQuadrature;
                break;

            case None:
            default:
                type = SparkRelativeEncoder.Type.kNoSensor;
                break;
        }

        this.useAbsoluteEncoder = false;
        this.wrappedRelativeEncoder = this.wrappedObject.getEncoder(type, resolution);
    }

    public void set(double value)
    {
        this.set(this.currentMode, value, 0.0);
    }

    public void set(double value, double feedForward)
    {
        this.set(this.currentMode, value, feedForward);
    }

    public void set(SparkMaxControlMode controlMode, double value)
    {
        this.set(controlMode, value, 0.0);
    }

    public void set(SparkMaxControlMode controlMode, double value, double feedForward)
    {
        if (controlMode == SparkMaxControlMode.PercentOutput)
        {
            this.wrappedObject.set(value);
            return;
        }

        this.ensurePidController();

        CANSparkBase.ControlType controlType;
        switch (controlMode)
        {
            case Position:
                controlType = CANSparkBase.ControlType.kPosition;
                break;

            case SmartMotionPosition:
                controlType = CANSparkBase.ControlType.kSmartMotion;
                break;

            case Velocity:
                controlType = CANSparkBase.ControlType.kVelocity;
                break;

            case Voltage:
                controlType = CANSparkBase.ControlType.kVoltage;
                break;

            default:
            case PercentOutput:
                throw new RuntimeException("unexpected control mode " + controlMode);
        }

        RevErrorCodeHelper.printError(
            this.pidController.setReference(value, controlType, selectedSlot, feedForward, ArbFFUnits.kPercentOut),
            "SparkMaxWrapper.set");
    }

    public void follow(ISparkMax sparkMax)
    {
        RevErrorCodeHelper.printError(
            this.wrappedObject.follow(((SparkMaxWrapper)sparkMax).wrappedObject),
            "SparkMaxWrapper.follow");
    }

    public void setFeedbackFramePeriod(SparkMaxPeriodicFrameType frameType, int periodMS)
    {
        CANSparkLowLevel.PeriodicFrame type = CANSparkLowLevel.PeriodicFrame.kStatus0;
        switch (frameType)
        {
            case Status0:
                type = CANSparkLowLevel.PeriodicFrame.kStatus0;
                break;
            case Status1:
                type = CANSparkLowLevel.PeriodicFrame.kStatus1;
                break;
            case Status2:
                type = CANSparkLowLevel.PeriodicFrame.kStatus2;
                break;
            case Status4:
                type = CANSparkLowLevel.PeriodicFrame.kStatus3;
                break;
            case Status3:
                type = CANSparkLowLevel.PeriodicFrame.kStatus4;
                break;
            case Status5:
                type = CANSparkLowLevel.PeriodicFrame.kStatus5;
                break;
            case Status6:
                type = CANSparkLowLevel.PeriodicFrame.kStatus6;
                break;
        }

        RevErrorCodeHelper.printError(
            this.wrappedObject.setPeriodicFramePeriod(type, periodMS),
            "SparkMaxWrapper.setFeedbackFramePeriod");
    }

    public void setEncoderAverageDepth(int windowSize)
    {
        if (this.useAbsoluteEncoder)
        {
            RevErrorCodeHelper.printError(
                this.wrappedAbsoluteEncoder.setAverageDepth(windowSize),
                "SparkMaxWrapper.setVelocityMeasurements-setAverageDepth");
        }
        else // if (!this.useAbsoluteEncoder)
        {
            RevErrorCodeHelper.printError(
                this.wrappedRelativeEncoder.setAverageDepth(windowSize),
                "SparkMaxWrapper.setVelocityMeasurements-setAverageDepth");
        }
    }
    public void setVelocityMeasurementPeriod(int periodMS)
    {
        // only supported for relative encoders
        if (!this.useAbsoluteEncoder)
        {
            RevErrorCodeHelper.printError(
                this.wrappedRelativeEncoder.setMeasurementPeriod(periodMS),
                "SparkMaxWrapper.setVelocityMeasurements-setMeasurementPeriod");
        }
    }

    public void setSelectedSlot(int slotId)
    {
        this.selectedSlot = slotId;
    }

    public void setPIDF(double p, double i, double d, double f, int slotId)
    {
        this.ensurePidController();

        RevErrorCodeHelper.printError(
            this.pidController.setP(p, slotId),
            "SparkMaxWrapper.setPIDF-p");
        RevErrorCodeHelper.printError(
            this.pidController.setI(i, slotId),
            "SparkMaxWrapper.setPIDF-i");
        RevErrorCodeHelper.printError(
            this.pidController.setD(d, slotId),
            "SparkMaxWrapper.setPIDF-d");
        RevErrorCodeHelper.printError(
            this.pidController.setFF(f, slotId),
            "SparkMaxWrapper.setPIDF-f");
    }

    public void setPIDF(double p, double i, double d, double f, double minOutput, double maxOutput, int slotId)
    {
        this.ensurePidController();

        RevErrorCodeHelper.printError(
            this.pidController.setP(p, slotId),
            "SparkMaxWrapper.setPIDF-p");
        RevErrorCodeHelper.printError(
            this.pidController.setI(i, slotId),
            "SparkMaxWrapper.setPIDF-i");
        RevErrorCodeHelper.printError(
            this.pidController.setD(d, slotId),
            "SparkMaxWrapper.setPIDF-d");
        RevErrorCodeHelper.printError(
            this.pidController.setFF(f, slotId),
            "SparkMaxWrapper.setPIDF-f");
        RevErrorCodeHelper.printError(
            this.pidController.setOutputRange(minOutput, maxOutput),
            "SparkMaxWrapper.setPIDF-output");
    }

    public void setPIDF(double p, double i, double d, double f, int izone, int slotId)
    {
        this.ensurePidController();

        RevErrorCodeHelper.printError(
            this.pidController.setP(p, slotId),
            "SparkMaxWrapper.setPIDF-p");
        RevErrorCodeHelper.printError(
            this.pidController.setI(i, slotId),
            "SparkMaxWrapper.setPIDF-i");
        RevErrorCodeHelper.printError(
            this.pidController.setD(d, slotId),
            "SparkMaxWrapper.setPIDF-d");
        RevErrorCodeHelper.printError(
            this.pidController.setFF(f, slotId),
            "SparkMaxWrapper.setPIDF-f");
        RevErrorCodeHelper.printError(
            this.pidController.setIZone(izone, slotId),
            "SparkMaxWrapper.setPIDF-izone");
    }

    public void setPIDF(double p, double i, double d, double f, int izone, double minOutput, double maxOutput, int slotId)
    {
        this.ensurePidController();

        RevErrorCodeHelper.printError(
            this.pidController.setP(p, slotId),
            "SparkMaxWrapper.setPIDF-p");
        RevErrorCodeHelper.printError(
            this.pidController.setI(i, slotId),
            "SparkMaxWrapper.setPIDF-i");
        RevErrorCodeHelper.printError(
            this.pidController.setD(d, slotId),
            "SparkMaxWrapper.setPIDF-d");
        RevErrorCodeHelper.printError(
            this.pidController.setFF(f, slotId),
            "SparkMaxWrapper.setPIDF-f");
        RevErrorCodeHelper.printError(
            this.pidController.setIZone(izone, slotId),
            "SparkMaxWrapper.setPIDF-izone");
        RevErrorCodeHelper.printError(
            this.pidController.setOutputRange(minOutput, maxOutput),
            "SparkMaxWrapper.setPIDF-output");
    }

    public void setPIDFSmartMotion(double p, double i, double d, double f, int izone, int velocity, int acceleration, int slotId)
    {
        this.ensurePidController();

        RevErrorCodeHelper.printError(
            this.pidController.setP(p, slotId),
            "SparkMaxWrapper.setPIDFSmartMotion-p");
        RevErrorCodeHelper.printError(
            this.pidController.setI(i, slotId),
            "SparkMaxWrapper.setPIDFSmartMotion-i");
        RevErrorCodeHelper.printError(
            this.pidController.setD(d, slotId),
            "SparkMaxWrapper.setPIDFSmartMotion-d");
        RevErrorCodeHelper.printError(
            this.pidController.setFF(f, slotId),
            "SparkMaxWrapper.setPIDFSmartMotion-f");
        RevErrorCodeHelper.printError(
            this.pidController.setIZone(izone, slotId),
            "SparkMaxWrapper.setPIDFSmartMotion-izone");
        RevErrorCodeHelper.printError(
            this.pidController.setSmartMotionMaxVelocity(velocity, slotId),
            "SparkMaxWrapper.setPIDFSmartMotion-maxvelocity");
        RevErrorCodeHelper.printError(
            this.pidController.setSmartMotionMaxAccel(acceleration, slotId),
            "SparkMaxWrapper.setPIDFSmartMotion-maxaccel");
        RevErrorCodeHelper.printError(
            this.pidController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, slotId),
            "SparkMaxWrapper.setPIDFSmartMotion-accelStrategy");
    }

    public void setPIDFSmartMotion(double p, double i, double d, double f, int izone, int velocity, int acceleration, double minOutput, double maxOutput, int slotId)
    {
        this.ensurePidController();

        RevErrorCodeHelper.printError(
            this.pidController.setP(p, slotId),
            "SparkMaxWrapper.setPIDFSmartMotion-p");
        RevErrorCodeHelper.printError(
            this.pidController.setI(i, slotId),
            "SparkMaxWrapper.setPIDFSmartMotion-i");
        RevErrorCodeHelper.printError(
            this.pidController.setD(d, slotId),
            "SparkMaxWrapper.setPIDFSmartMotion-d");
        RevErrorCodeHelper.printError(
            this.pidController.setFF(f, slotId),
            "SparkMaxWrapper.setPIDFSmartMotion-f");
        RevErrorCodeHelper.printError(
            this.pidController.setIZone(izone, slotId),
            "SparkMaxWrapper.setPIDFSmartMotion-izone");
        RevErrorCodeHelper.printError(
            this.pidController.setSmartMotionMaxVelocity(velocity, slotId),
            "SparkMaxWrapper.setPIDFSmartMotion-maxvelocity");
        RevErrorCodeHelper.printError(
            this.pidController.setSmartMotionMaxAccel(acceleration, slotId),
            "SparkMaxWrapper.setPIDFSmartMotion-maxaccel");
        RevErrorCodeHelper.printError(
            this.pidController.setOutputRange(minOutput, maxOutput),
            "SparkMaxWrapper.setPIDFSmartMotion-output");
        RevErrorCodeHelper.printError(
            this.pidController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, slotId),
            "SparkMaxWrapper.setPIDFSmartMotion-accelStrategy");
    }

    public void setForwardLimitSwitch(boolean enabled, boolean normallyOpen)
    {
        SparkLimitSwitch.Type polarity = SparkLimitSwitch.Type.kNormallyClosed;
        if (normallyOpen)
        {
            polarity = SparkLimitSwitch.Type.kNormallyOpen;
        }

        this.wrappedFwdLimitSwitch = this.wrappedObject.getForwardLimitSwitch(polarity);
        RevErrorCodeHelper.printError(
            this.wrappedFwdLimitSwitch.enableLimitSwitch(enabled),
            "SparkMaxWrapper.setForwardLimitSwitch");
    }

    public void setReverseLimitSwitch(boolean enabled, boolean normallyOpen)
    {
        SparkLimitSwitch.Type polarity = SparkLimitSwitch.Type.kNormallyClosed;
        if (normallyOpen)
        {
            polarity = SparkLimitSwitch.Type.kNormallyOpen;
        }

        this.wrappedRevLimitSwitch = this.wrappedObject.getReverseLimitSwitch(polarity);
        RevErrorCodeHelper.printError(
            this.wrappedRevLimitSwitch.enableLimitSwitch(enabled),
            "SparkMaxWrapper.setReverseLimitSwitch");
    }

    public void setInvertOutput(boolean invert)
    {
        this.wrappedObject.setInverted(invert);
    }

    public void setInvertSensor(boolean invert)
    {
        if (this.useAbsoluteEncoder)
        {
            RevErrorCodeHelper.printError(
                this.wrappedAbsoluteEncoder.setInverted(invert),
                "SparkMaxWrapper.setInvertSensor");
        }
        else // if (!this.useAbsoluteEncoder)
        {
            RevErrorCodeHelper.printError(
                this.wrappedRelativeEncoder.setInverted(invert),
                "SparkMaxWrapper.setInvertSensor");
        }
    }

    public void setNeutralMode(MotorNeutralMode neutralMode)
    {
        IdleMode mode;
        if (neutralMode == MotorNeutralMode.Brake)
        {
            mode = IdleMode.kBrake;
        }
        else
        {
            mode = IdleMode.kCoast;
        }

        RevErrorCodeHelper.printError(
            this.wrappedObject.setIdleMode(mode),
            "SparkMaxWrapper.setNeutralMode");
    }

    public void setCurrentLimit(int stallLimit, int freeLimit, int limitRPM)
    {
        RevErrorCodeHelper.printError(
            this.wrappedObject.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM),
            "SparkMaxWrapper.setCurrentLimit");
    }

    public void stop()
    {
        this.wrappedObject.stopMotor();
    }

    public void setAbsoluteOffset(double zeroOffset)
    {
        if (this.useAbsoluteEncoder)
        {
            RevErrorCodeHelper.printError(
                this.wrappedAbsoluteEncoder.setZeroOffset(zeroOffset),
                "SparkMaxWrapper.setAbsoluteOffset");
        }
    }

    public void setPosition(double position)
    {
        if (this.useAbsoluteEncoder)
        {
            // update zero offset to current location
            double currentPosition = this.wrappedAbsoluteEncoder.getZeroOffset() + this.wrappedAbsoluteEncoder.getPosition();
            RevErrorCodeHelper.printError(
                this.wrappedAbsoluteEncoder.setZeroOffset(currentPosition - position),
                "SparkMaxWrapper.setPosition");
        }
        else // if (!this.useAbsoluteEncoder)
        {
            RevErrorCodeHelper.printError(
                this.wrappedRelativeEncoder.setPosition(position),
                "SparkMaxWrapper.setPosition");
        }
    }

    public void reset()
    {
        this.setPosition(0.0);
    }

    public void burnFlash()
    {
        RevErrorCodeHelper.printError(
            this.wrappedObject.burnFlash(),
            "SparkMaxWrapper.burnFlash");
    }

    public void setPositionConversionFactor(double ratio)
    {
        if (this.useAbsoluteEncoder)
        {
            RevErrorCodeHelper.printError(
                this.wrappedAbsoluteEncoder.setPositionConversionFactor(ratio),
                "SparkMaxWrapper.setPositionConversionFactor");
        }
        else // if (!this.useAbsoluteEncoder)
        {
            RevErrorCodeHelper.printError(
                this.wrappedRelativeEncoder.setPositionConversionFactor(ratio),
                "SparkMaxWrapper.setPositionConversionFactor");
        }
    }

    public void setVelocityConversionFactor(double ratio)
    {
        if (this.useAbsoluteEncoder)
        {
            RevErrorCodeHelper.printError(
                this.wrappedAbsoluteEncoder.setVelocityConversionFactor(ratio),
                "SparkMaxWrapper.setVelocityConversionFactor");
        }
        else // if (!this.useAbsoluteEncoder)
        {
            RevErrorCodeHelper.printError(
                this.wrappedRelativeEncoder.setVelocityConversionFactor(ratio),
                "SparkMaxWrapper.setVelocityConversionFactor");
        }
    }

    public void setPositionPIDWrappingSettings(boolean enable, double minInput, double maxInput)
    {
        this.ensurePidController();

        RevErrorCodeHelper.printError(
            this.pidController.setPositionPIDWrappingEnabled(enable),
            "SparkMaxWrapper.setPositionPIDWrappingSettings");
        RevErrorCodeHelper.printError(
            this.pidController.setPositionPIDWrappingMinInput(minInput),
            "SparkMaxWrapper.setPositionPIDWrappingSettings");
        RevErrorCodeHelper.printError(
            this.pidController.setPositionPIDWrappingMaxInput(maxInput),
            "SparkMaxWrapper.setPositionPIDWrappingSettings");
    }

    public double getPosition()
    {
        if (this.useAbsoluteEncoder)
        {
            return this.wrappedAbsoluteEncoder.getPosition();
        }
        else // if (!this.useAbsoluteEncoder)
        {
            return this.wrappedRelativeEncoder.getPosition();
        }
    }

    public double getVelocity()
    {
        // NOTE: SparkMAX Absolute encoder provides velocity in Rotations per Second,
        // but SparkMAX Relative/Alternative encoder provides velocity in Rotations per Minute.
        if (this.useAbsoluteEncoder)
        {
            return this.wrappedAbsoluteEncoder.getVelocity();
        }
        else // if (!this.useAbsoluteEncoder)
        {
            return this.wrappedRelativeEncoder.getVelocity();
        }
    }

    public double getOutput()
    {
        return this.wrappedObject.getAppliedOutput();
    }

    public boolean getForwardLimitSwitchStatus()
    {
        if (this.wrappedFwdLimitSwitch == null)
        {
            return false;
        }

        return this.wrappedFwdLimitSwitch.isPressed();
    }

    public boolean getReverseLimitSwitchStatus()
    {
        if (this.wrappedRevLimitSwitch == null)
        {
            return false;
        }

        return this.wrappedRevLimitSwitch.isPressed();
    }

    private void ensurePidController()
    {
        if (this.pidController == null)
        {
            this.pidController = this.wrappedObject.getPIDController();
            if (this.useAbsoluteEncoder)
            {
                this.pidController.setFeedbackDevice(this.wrappedAbsoluteEncoder);
            }
            else
            {
                this.pidController.setFeedbackDevice(this.wrappedRelativeEncoder);
            }
        }
    }
}
