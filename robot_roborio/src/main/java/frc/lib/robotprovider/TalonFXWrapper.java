package frc.lib.robotprovider;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

import frc.lib.helpers.ExceptionHelpers;

public class TalonFXWrapper implements ITalonFX
{
    private static final double timeoutSecs = 0.05;

    private static final NeutralOut stop = new NeutralOut();

    final TalonFX wrappedObject;

    private final String talonId;

    private TalonFXConfigurator currentConfigurator;

    private TalonFXControlMode controlMode;
    private ControlRequest currentControlRequest;
    private int selectedSlot;
    private boolean useVoltageCompensation;
    private double maxVoltage;

    private StatusSignal<Double> position;
    private StatusSignal<Double> velocity;
    private StatusSignal<Double> error;

    private StatusSignal<ForwardLimitValue> forwardLimitSwitch;
    private StatusSignal<ReverseLimitValue> reverseLimitSwitch;
    public TalonFXWrapper(int deviceNumber)
    {
        this(new TalonFX(deviceNumber), String.format("TalonFX %d", deviceNumber));
    }

    public TalonFXWrapper(int deviceNumber, String canbus)
    {
        this(new TalonFX(deviceNumber, canbus), String.format("TalonFX %s-%d", canbus, deviceNumber));
    }

    private TalonFXWrapper(TalonFX wrappedObject, String talonId)
    {
        this.wrappedObject = wrappedObject;
        this.talonId = talonId;

        this.controlMode = TalonFXControlMode.Neutral;
        this.currentControlRequest = TalonFXWrapper.stop;
        this.selectedSlot = 0;
    }

    public void set(double value)
    {
        this.internalSet(this.controlMode, this.selectedSlot, value, 0.0);
    }

    public void set(double value, double feedForward)
    {
        this.internalSet(this.controlMode, this.selectedSlot, value, feedForward);
    }

    public void set(TalonFXControlMode mode, double value)
    {
        this.internalSet(mode, this.selectedSlot, value, 0.0);
    }

    public void set(TalonFXControlMode mode, double value, double feedForward)
    {
        this.internalSet(mode, this.selectedSlot, value, feedForward);
    }

    public void set(TalonFXControlMode mode, int slotId, double value)
    {
        this.internalSet(mode, slotId, value, 0.0);
    }

    public void set(TalonFXControlMode mode, int slotId, double value, double feedForward)
    {
        this.internalSet(mode, slotId, value, feedForward);
    }

    private void internalSet(TalonFXControlMode mode, int slotId, double value, double feedForward)
    {
        switch (mode)
        {
            case PercentOutput:
                ExceptionHelpers.Assert(feedForward == 0.0, "%s: Don't expect to see feedForward for PercentOutput", this.talonId);
                if (this.useVoltageCompensation)
                {
                    VoltageOut voRequest;
                    if (this.currentControlRequest instanceof VoltageOut)
                    {
                        voRequest = (VoltageOut)this.currentControlRequest;
                        voRequest.withOutput(value * this.maxVoltage);
                    }
                    else
                    {
                        voRequest = new VoltageOut(value * this.maxVoltage);
                    }

                    this.wrappedObject.setControl(voRequest);
                }
                else
                {
                    DutyCycleOut dcRequest;
                    if (this.currentControlRequest instanceof DutyCycleOut)
                    {
                        dcRequest = (DutyCycleOut)this.currentControlRequest;
                        dcRequest.withOutput(value);
                    }
                    else
                    {
                        dcRequest = new DutyCycleOut(value);
                    }

                    this.wrappedObject.setControl(dcRequest);
                }

                break;

            case Position:
                if (this.useVoltageCompensation)
                {
                    PositionVoltage pvRequest;
                    if (this.currentControlRequest instanceof PositionVoltage)
                    {
                        pvRequest = (PositionVoltage)this.currentControlRequest;
                        pvRequest.withPosition(value);
                    }
                    else
                    {
                        pvRequest = new PositionVoltage(value);
                    }

                    pvRequest.withFeedForward(feedForward);
                    pvRequest.withSlot(slotId);
                    this.wrappedObject.setControl(pvRequest);
                }
                else
                {
                    PositionDutyCycle pdcRequest;
                    if (this.currentControlRequest instanceof PositionDutyCycle)
                    {
                        pdcRequest = (PositionDutyCycle)this.currentControlRequest;
                        pdcRequest.withPosition(value);
                    }
                    else
                    {
                        pdcRequest = new PositionDutyCycle(value);
                    }

                    pdcRequest.withFeedForward(feedForward);
                    pdcRequest.withSlot(slotId);
                    this.wrappedObject.setControl(pdcRequest);
                }

                break;

            case Velocity:
                if (this.useVoltageCompensation)
                {
                    VelocityVoltage vvRequest;
                    if (this.currentControlRequest instanceof VelocityVoltage)
                    {
                        vvRequest = (VelocityVoltage)this.currentControlRequest;
                        vvRequest.withVelocity(value);
                    }
                    else
                    {
                        vvRequest = new VelocityVoltage(value);
                    }

                    vvRequest.withFeedForward(feedForward);
                    vvRequest.withSlot(slotId);
                    this.wrappedObject.setControl(vvRequest);
                }
                else
                {
                    VelocityDutyCycle vdcRequest;
                    if (this.currentControlRequest instanceof VelocityDutyCycle)
                    {
                        vdcRequest = (VelocityDutyCycle)this.currentControlRequest;
                        vdcRequest.withVelocity(value);
                    }
                    else
                    {
                        vdcRequest = new VelocityDutyCycle(value);
                    }

                    vdcRequest.withFeedForward(feedForward);
                    vdcRequest.withSlot(slotId);
                    this.wrappedObject.setControl(vdcRequest);
                }

                break;

            case MotionMagicPosition:
                if (this.useVoltageCompensation)
                {
                    MotionMagicVoltage mmvRequest;
                    if (this.currentControlRequest instanceof MotionMagicVoltage)
                    {
                        mmvRequest = (MotionMagicVoltage)this.currentControlRequest;
                        mmvRequest.withPosition(value);
                    }
                    else
                    {
                        mmvRequest = new MotionMagicVoltage(value);
                    }

                    mmvRequest.withFeedForward(feedForward);
                    mmvRequest.withSlot(slotId);
                    this.wrappedObject.setControl(mmvRequest);
                }
                else
                {
                    MotionMagicDutyCycle mmdcRequest;
                    if (this.currentControlRequest instanceof MotionMagicDutyCycle)
                    {
                        mmdcRequest = (MotionMagicDutyCycle)this.currentControlRequest;
                        mmdcRequest.withPosition(value);
                    }
                    else
                    {
                        mmdcRequest = new MotionMagicDutyCycle(value);
                    }

                    mmdcRequest.withFeedForward(feedForward);
                    mmdcRequest.withSlot(slotId);
                    this.wrappedObject.setControl(mmdcRequest);
                }

                break;

            case Coast:
                ExceptionHelpers.Assert(feedForward == 0.0, "%s: Don't expect to see feedForward for Coast", this.talonId);
                CoastOut coRequest;
                if (this.currentControlRequest instanceof CoastOut)
                {
                    coRequest = (CoastOut)this.currentControlRequest;
                }
                else
                {
                    coRequest = new CoastOut();
                }

                this.wrappedObject.setControl(coRequest);
                break;

            case StaticBrake:
                ExceptionHelpers.Assert(feedForward == 0.0, "%s: Don't expect to see feedForward for StaticBrake", this.talonId);
                StaticBrake sbRequest;
                if (this.currentControlRequest instanceof StaticBrake)
                {
                    sbRequest = (StaticBrake)this.currentControlRequest;
                }
                else
                {
                    sbRequest = new StaticBrake();
                }

                this.wrappedObject.setControl(sbRequest);
                break;

            default:
            case Neutral:
                ExceptionHelpers.Assert(feedForward == 0.0, "%s: Don't expect to see feedForward for Neutral/default", this.talonId);
                NeutralOut noRequest;
                if (this.currentControlRequest instanceof NeutralOut)
                {
                    noRequest = (NeutralOut)this.currentControlRequest;
                }
                else
                {
                    noRequest = TalonFXWrapper.stop;
                }

                this.wrappedObject.setControl(noRequest);
                break;
        }
    }

    public void follow(ITalonFX talonFX)
    {
        this.controlMode = TalonFXControlMode.Follower;
        this.currentControlRequest = new StrictFollower(((TalonFXWrapper)talonFX).wrappedObject.getDeviceID());
        this.wrappedObject.setControl((StrictFollower)this.currentControlRequest);
    }

    public void follow(ITalonFX talonFX, boolean invertDirection)
    {
        this.controlMode = TalonFXControlMode.Follower;
        this.currentControlRequest = new Follower(((TalonFXWrapper)talonFX).wrappedObject.getDeviceID(), invertDirection);
        this.wrappedObject.setControl((Follower)this.currentControlRequest);
    }

    public void setControlMode(TalonFXControlMode mode)
    {
        if (this.controlMode != mode)
        {
            this.controlMode = mode;
            switch (this.controlMode)
            {
                case PercentOutput:
                    this.currentControlRequest = this.useVoltageCompensation ? new VoltageOut(0.0) : new DutyCycleOut(0.0);
                    break;

                case Follower:
                    this.currentControlRequest = new StrictFollower(-1);
                    break;

                case Position:
                    if (this.useVoltageCompensation)
                    {
                        this.currentControlRequest = (new PositionVoltage(0.0)).withSlot(this.selectedSlot);
                    }
                    else
                    {
                        this.currentControlRequest = (new PositionDutyCycle(0.0)).withSlot(this.selectedSlot);
                    }

                    break;

                case Velocity:
                    if (this.useVoltageCompensation)
                    {
                        this.currentControlRequest = (new VelocityVoltage(0.0)).withSlot(this.selectedSlot);
                    }
                    else
                    {
                        this.currentControlRequest = (new VelocityDutyCycle(0.0)).withSlot(this.selectedSlot);
                    }

                    break;

                case MotionMagicPosition:
                    if (this.useVoltageCompensation)
                    {
                        this.currentControlRequest = (new MotionMagicVoltage(0.0)).withSlot(this.selectedSlot);
                    }
                    else
                    {
                        this.currentControlRequest = (new MotionMagicDutyCycle(0.0)).withSlot(this.selectedSlot);
                    }

                    break;

                case Coast:
                    this.currentControlRequest = new CoastOut();
                    break;

                case StaticBrake:
                    this.currentControlRequest = new StaticBrake();
                    break;

                default:
                case Neutral:
                    this.currentControlRequest = TalonFXWrapper.stop;
                    break;
            }
        }
    }

    private void ensureConfigurator()
    {
        if (this.currentConfigurator == null)
        {
            this.currentConfigurator = this.wrappedObject.getConfigurator();
        }
    }

    public void clearRemoteSensor()
    {
        this.ensureConfigurator();

        // apply default feedback config settings
        CTREStatusCodeHelper.printError(
            this.currentConfigurator.apply(new FeedbackConfigs(), TalonFXWrapper.timeoutSecs),
            this.talonId,
            "TalonFX.clearRemoteSensor-apply");
    }

    public void setRemoteSensor(int sensorId, double ratio)
    {
        this.ensureConfigurator();

        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        feedbackConfigs.FeedbackRemoteSensorID = sensorId;
        feedbackConfigs.RotorToSensorRatio = ratio;

        CTREStatusCodeHelper.printError(
            this.currentConfigurator.apply(feedbackConfigs, TalonFXWrapper.timeoutSecs),
            this.talonId,
            "TalonFX.updateRemoteSensor-apply");
    }

    public void setFeedbackUpdateRate(double frequencyHz)
    {
        if (this.position == null)
        {
            this.position = this.wrappedObject.getPosition();
        }

        if (this.velocity == null)
        {
            this.velocity = this.wrappedObject.getVelocity();
        }

        CTREStatusCodeHelper.printError(
            this.position.setUpdateFrequency(frequencyHz, TalonFXWrapper.timeoutSecs),
            this.talonId,
            "TalonFX.setFeedbackUpdateRate-Position");
        CTREStatusCodeHelper.printError(
            this.velocity.setUpdateFrequency(frequencyHz, TalonFXWrapper.timeoutSecs),
            this.talonId,
            "TalonFX.setFeedbackFramePeriod-Velocity");
    }

    public void setErrorUpdateRate(double frequencyHz)
    {
        if (this.error == null)
        {
            this.error = this.wrappedObject.getClosedLoopError();
        }

        CTREStatusCodeHelper.printError(
            this.error.setUpdateFrequency(frequencyHz, TalonFXWrapper.timeoutSecs),
            this.talonId,
            "TalonFX.setErrorUpdateRate");
    }

    public void setForwardLimitSwitchUpdateRate(double frequencyHz)
    {
        if (this.forwardLimitSwitch == null)
        {
            this.forwardLimitSwitch = this.wrappedObject.getForwardLimit();
        }

        CTREStatusCodeHelper.printError(
            this.forwardLimitSwitch.setUpdateFrequency(frequencyHz, TalonFXWrapper.timeoutSecs),
            this.talonId,
            "TalonFXWrapper.setForwardLimitSwitchUpdateRate");
    }

    public void setReverseLimitSwitchUpdateRate(double frequencyHz)
    {
        if (this.reverseLimitSwitch == null)
        {
            this.reverseLimitSwitch = this.wrappedObject.getReverseLimit();
        }

        CTREStatusCodeHelper.printError(
            this.reverseLimitSwitch.setUpdateFrequency(frequencyHz, TalonFXWrapper.timeoutSecs),
            this.talonId,
            "TalonFXWrapper.setReverseLimitSwitchUpdateRate");
    }

    public void optimizeCanbus()
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.optimizeBusUtilization(),
            this.talonId,
            "TalonFX.optimizeCanbus");
    }

    public void setSelectedSlot(int slotId)
    {
        this.selectedSlot = slotId;
        switch (this.controlMode)
        {
            case Position:
                if (this.useVoltageCompensation)
                {
                    PositionVoltage pvRequest;
                    if (this.currentControlRequest instanceof PositionVoltage)
                    {
                        pvRequest = (PositionVoltage)this.currentControlRequest;
                    }
                    else
                    {
                        pvRequest = new PositionVoltage(0.0);
                        this.currentControlRequest = pvRequest;
                    }
    
                    pvRequest.withSlot(slotId);
                }
                else
                {
                    PositionDutyCycle pdcRequest;
                    if (this.currentControlRequest instanceof PositionDutyCycle)
                    {
                        pdcRequest = (PositionDutyCycle)this.currentControlRequest;
                    }
                    else
                    {
                        pdcRequest = new PositionDutyCycle(0.0);
                        this.currentControlRequest = pdcRequest;
                    }

                    pdcRequest.withSlot(slotId);
                }

                break;

            case Velocity:
                if (this.useVoltageCompensation)
                {
                    VelocityVoltage vvRequest;
                    if (this.currentControlRequest instanceof VelocityVoltage)
                    {
                        vvRequest = (VelocityVoltage)this.currentControlRequest;
                    }
                    else
                    {
                        vvRequest = new VelocityVoltage(0.0);
                        this.currentControlRequest = vvRequest;
                    }
    
                    vvRequest.withSlot(slotId);
                }
                else
                {
                    VelocityDutyCycle vvdcRequest;
                    if (this.currentControlRequest instanceof VelocityDutyCycle)
                    {
                        vvdcRequest = (VelocityDutyCycle)this.currentControlRequest;
                    }
                    else
                    {
                        vvdcRequest = new VelocityDutyCycle(0.0);
                        this.currentControlRequest = vvdcRequest;
                    }

                    vvdcRequest.withSlot(slotId);
                }

                break;

            case MotionMagicPosition:
                if (this.useVoltageCompensation)
                {
                    MotionMagicVoltage mmvRequest;
                    if (this.currentControlRequest instanceof MotionMagicVoltage)
                    {
                        mmvRequest = (MotionMagicVoltage)this.currentControlRequest;
                    }
                    else
                    {
                        mmvRequest = new MotionMagicVoltage(0.0);
                        this.currentControlRequest = mmvRequest;
                    }
    
                    mmvRequest.withSlot(slotId);
                }
                else
                {
                    MotionMagicDutyCycle mmdcRequest;
                    if (this.currentControlRequest instanceof MotionMagicDutyCycle)
                    {
                        mmdcRequest = (MotionMagicDutyCycle)this.currentControlRequest;
                    }
                    else
                    {
                        mmdcRequest = new MotionMagicDutyCycle(0.0);
                        this.currentControlRequest = mmdcRequest;
                    }

                    mmdcRequest.withSlot(slotId);
                }

                break;

            default:
                // no relevant slot for other scenarios...
                break;
        }
    }

    public void setPIDF(double p, double i, double d, double f, int slotId)
    {
        this.ensureConfigurator();

        switch (slotId)
        {
            case 0:
                Slot0Configs slot0Configs = new Slot0Configs();
                slot0Configs.kP = p;
                slot0Configs.kI = i;
                slot0Configs.kD = d;
                slot0Configs.kV = f;
                CTREStatusCodeHelper.printError(
                    this.currentConfigurator.apply(slot0Configs, TalonFXWrapper.timeoutSecs),
                    this.talonId,
                    "TalonFXWrapper.setPIDF");
                break;

            case 1:
                Slot1Configs slot1Configs = new Slot1Configs();
                slot1Configs.kP = p;
                slot1Configs.kI = i;
                slot1Configs.kD = d;
                slot1Configs.kV = f;
                CTREStatusCodeHelper.printError(
                    this.currentConfigurator.apply(slot1Configs, TalonFXWrapper.timeoutSecs),
                    this.talonId,
                    "TalonFXWrapper.setPIDF");
                break;

            default:
            case 2:
                Slot2Configs slot2Configs = new Slot2Configs();
                slot2Configs.kP = p;
                slot2Configs.kI = i;
                slot2Configs.kD = d;
                slot2Configs.kV = f;
                CTREStatusCodeHelper.printError(
                    this.currentConfigurator.apply(slot2Configs, TalonFXWrapper.timeoutSecs),
                    this.talonId,
                    "TalonFXWrapper.setPIDF");
                break;
        }
    }

    public void setMotionMagicPIDVS(double p, double i, double d, double v, double s, double cruiseVelocity, double maxAcceleration, double maxJerk, int slotId)
    {
        this.ensureConfigurator();

        switch (slotId)
        {
            case 0:
                Slot0Configs slot0Configs = new Slot0Configs();
                slot0Configs.kP = p;
                slot0Configs.kI = i;
                slot0Configs.kD = d;
                slot0Configs.kV = v;
                slot0Configs.kS = s;
                CTREStatusCodeHelper.printError(
                    this.currentConfigurator.apply(slot0Configs, TalonFXWrapper.timeoutSecs),
                    this.talonId,
                    "TalonFXWrapper.setMotionMagicPIDVS-PID");
                break;

            case 1:
                Slot1Configs slot1Configs = new Slot1Configs();
                slot1Configs.kP = p;
                slot1Configs.kI = i;
                slot1Configs.kD = d;
                slot1Configs.kV = v;
                slot1Configs.kS = s;
                CTREStatusCodeHelper.printError(
                    this.currentConfigurator.apply(slot1Configs, TalonFXWrapper.timeoutSecs),
                    this.talonId,
                    "TalonFXWrapper.setMotionMagicPIDVS-PID");
                break;

            default:
            case 2:
                Slot2Configs slot2Configs = new Slot2Configs();
                slot2Configs.kP = p;
                slot2Configs.kI = i;
                slot2Configs.kD = d;
                slot2Configs.kV = v;
                slot2Configs.kS = s;
                CTREStatusCodeHelper.printError(
                    this.currentConfigurator.apply(slot2Configs, TalonFXWrapper.timeoutSecs),
                    this.talonId,
                    "TalonFXWrapper.setMotionMagicPIDVS-PID");
                break;
        }

        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = cruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = maxAcceleration;
        motionMagicConfigs.MotionMagicJerk = maxJerk;
        CTREStatusCodeHelper.printError(
            this.currentConfigurator.apply(motionMagicConfigs, TalonFXWrapper.timeoutSecs),
            this.talonId,
            "TalonFXWrapper.setMotionMagicPIDVS-MM");
    }

    public void updateLimitSwitchConfig(boolean forwardEnabled, boolean forwardNormallyOpen, boolean reverseEnabled, boolean reverseNormallyOpen)
    {
        this.updateLimitSwitchConfig(
            forwardEnabled,
            forwardNormallyOpen,
            false,
            0.0,
            reverseEnabled,
            reverseNormallyOpen,
            false,
            0.0);
    }

    public void updateLimitSwitchConfig(
        boolean forwardEnabled,
        boolean forwardNormallyOpen,
        boolean forwardReset,
        double forwardResetPosition,
        boolean reverseEnabled,
        boolean reverseNormallyOpen,
        boolean reverseReset,
        double reverseResetPosition)
    {
        this.ensureConfigurator();

        HardwareLimitSwitchConfigs hardwareLimitSwitchConfigs = new HardwareLimitSwitchConfigs();
        hardwareLimitSwitchConfigs.ForwardLimitEnable = forwardEnabled;
        hardwareLimitSwitchConfigs.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
        hardwareLimitSwitchConfigs.ForwardLimitType = forwardNormallyOpen ? ForwardLimitTypeValue.NormallyOpen : ForwardLimitTypeValue.NormallyClosed;
        hardwareLimitSwitchConfigs.ForwardLimitAutosetPositionEnable = forwardReset;
        hardwareLimitSwitchConfigs.ForwardLimitAutosetPositionValue = forwardResetPosition;
        hardwareLimitSwitchConfigs.ReverseLimitEnable = reverseEnabled;
        hardwareLimitSwitchConfigs.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
        hardwareLimitSwitchConfigs.ReverseLimitType = reverseNormallyOpen ? ReverseLimitTypeValue.NormallyOpen : ReverseLimitTypeValue.NormallyClosed;
        hardwareLimitSwitchConfigs.ReverseLimitAutosetPositionEnable = reverseReset;
        hardwareLimitSwitchConfigs.ReverseLimitAutosetPositionValue = reverseResetPosition;

        CTREStatusCodeHelper.printError(
            this.currentConfigurator.apply(hardwareLimitSwitchConfigs, TalonFXWrapper.timeoutSecs),
            this.talonId,
            "TalonFXWrapper.updateLimitSwitchConfig");

        if (forwardEnabled && this.forwardLimitSwitch == null)
        {
            this.forwardLimitSwitch = this.wrappedObject.getForwardLimit();
        }

        if (reverseEnabled && this.reverseLimitSwitch == null)
        {
            this.reverseLimitSwitch = this.wrappedObject.getReverseLimit();
        }
    }

    public void setMotorOutputSettings(boolean invert, MotorNeutralMode neutralMode)
    {
        this.ensureConfigurator();

        NeutralModeValue mode;
        if (neutralMode == MotorNeutralMode.Brake)
        {
            mode = NeutralModeValue.Brake;
        }
        else
        {
            mode = NeutralModeValue.Coast;
        }

        MotorOutputConfigs motorConfigs = new MotorOutputConfigs();

        CTREStatusCodeHelper.printError(
            this.currentConfigurator.refresh(motorConfigs, TalonFXWrapper.timeoutSecs),
            this.talonId,
            "TalonFXWrapper.setMotorSettings-refresh");

        motorConfigs.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        motorConfigs.NeutralMode = mode;

        CTREStatusCodeHelper.printError(
            this.currentConfigurator.apply(motorConfigs, TalonFXWrapper.timeoutSecs),
            this.talonId,
            "TalonFXWrapper.setMotorSettings-apply");
    }

    public void setVoltageCompensation(boolean enabled, double maxVoltage)
    {
        this.useVoltageCompensation = enabled;
        this.maxVoltage = maxVoltage;
    }

    public void setCurrentLimit(boolean enabled, double currentLimit, double triggerThresholdCurrent, double triggerThresholdTime)
    {
        this.setCurrentLimit(enabled, currentLimit, triggerThresholdCurrent, triggerThresholdTime, false, 0.0);
    }

    public void setCurrentLimit(boolean enabled, double currentLimit, double triggerThresholdCurrent, double triggerThresholdTime, boolean statorLimiting, double statorCurrentLimit)
    {
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.SupplyCurrentLimitEnable = enabled;
        currentLimitsConfigs.SupplyCurrentLimit = currentLimit;
        currentLimitsConfigs.SupplyCurrentThreshold = triggerThresholdCurrent;
        currentLimitsConfigs.SupplyTimeThreshold = triggerThresholdTime;
        currentLimitsConfigs.StatorCurrentLimitEnable = statorLimiting;
        currentLimitsConfigs.StatorCurrentLimit = statorCurrentLimit;

        CTREStatusCodeHelper.printError(
            this.currentConfigurator.apply(currentLimitsConfigs, TalonFXWrapper.timeoutSecs),
            this.talonId,
            "TalonFX.setSupplyCurrentLimit");
    }

    public void stop()
    {
        this.wrappedObject.setControl(TalonFXWrapper.stop);
    }

    public void setPosition(double position)
    {
        this.ensureConfigurator();

        CTREStatusCodeHelper.printError(
            this.currentConfigurator.setPosition(position, TalonFXWrapper.timeoutSecs),
            this.talonId,
            "TalonFX.setPosition");
    }

    public void reset()
    {
        this.setPosition(0.0);
    }

    public double getPosition()
    {
        if (this.position == null)
        {
            this.position = this.wrappedObject.getPosition();
        }

        this.position.refresh();
        CTREStatusCodeHelper.printError(
            this.position.getStatus(),
            this.talonId,
            "TalonFX.getPosition");

        return this.position.getValue();
    }

    public double getVelocity()
    {
        if (this.velocity == null)
        {
            this.velocity = this.wrappedObject.getVelocity();
        }

        this.velocity.refresh();
        CTREStatusCodeHelper.printError(
            this.velocity.getStatus(),
            this.talonId,
            "TalonFX.getVelocity");

        return this.velocity.getValue();
    }

    public double getError()
    {
        if (this.error == null)
        {
            this.error = this.wrappedObject.getClosedLoopError();
        }

        this.error.refresh();
        CTREStatusCodeHelper.printError(
            this.error.getStatus(),
            this.talonId,
            "TalonFX.getError");

        return this.error.getValue();
    }

    public boolean getForwardLimitSwitchClosed()
    {
        if (this.forwardLimitSwitch == null)
        {
            this.forwardLimitSwitch = this.wrappedObject.getForwardLimit();
        }

        this.forwardLimitSwitch.refresh();
        return this.forwardLimitSwitch.getValue() == ForwardLimitValue.ClosedToGround;
    }

    public boolean getReverseLimitSwitchClosed()
    {
        if (this.reverseLimitSwitch == null)
        {
            this.reverseLimitSwitch = this.wrappedObject.getReverseLimit();
        }

        this.reverseLimitSwitch.refresh();
        return this.reverseLimitSwitch.getValue() == ReverseLimitValue.ClosedToGround;
    }

    public TalonXLimitSwitchStatus getLimitSwitchStatus()
    {
        if (this.forwardLimitSwitch == null)
        {
            this.forwardLimitSwitch = this.wrappedObject.getForwardLimit();
        }

        if (this.reverseLimitSwitch == null)
        {
            this.reverseLimitSwitch = this.wrappedObject.getReverseLimit();
        }

        this.forwardLimitSwitch.refresh();
        this.reverseLimitSwitch.refresh();

        return new TalonXLimitSwitchStatus(
            this.forwardLimitSwitch.getValue() == ForwardLimitValue.ClosedToGround,
            this.reverseLimitSwitch.getValue() == ReverseLimitValue.ClosedToGround);
    }
}
