package frc.robot.driver.controltasks;

import frc.lib.controllers.PIDHandler;
import frc.lib.robotprovider.ITimer;
import frc.robot.TuningConstants;
import frc.robot.driver.*;

public abstract class VisionMoveAndTurnTaskBase extends VisionContinuousTurningTask
{
    public enum MoveSpeed
    {
        Normal,
        Fast,
        Slow,
    }

    public enum MoveType
    {
        AprilTagForward,
        AprilTagStrafe,
    }

    private final boolean driveBackwards;
    private final MoveType translateType;
    private final MoveSpeed moveSpeed;
    private final boolean verifyAngle;

    private PIDHandler movePIDHandler;

    /**
    * Initializes a new VisionAdvanceAndCenterTaskBase
    */
    protected VisionMoveAndTurnTaskBase(TurnType rotateType, MoveType translateType, MoveSpeed moveSpeed, boolean bestEffort, boolean verifyAngle, DigitalOperation visionOperation, boolean driveBackwards, boolean continuous)
    {
        super(false, rotateType, bestEffort, visionOperation, continuous);

        this.driveBackwards = driveBackwards;
        this.translateType = translateType;

        this.moveSpeed = moveSpeed;
        this.verifyAngle = verifyAngle;

        this.movePIDHandler = null;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        super.begin();

        this.setDigitalOperationState(DigitalOperation.DriveTrainUseRobotOrientation, true);
        switch (this.moveSpeed)
        {
            case Fast:
                this.movePIDHandler = new PIDHandler(
                    TuningConstants.VISION_FAST_MOVING_PID_KP,
                    TuningConstants.VISION_FAST_MOVING_PID_KI,
                    TuningConstants.VISION_FAST_MOVING_PID_KD,
                    TuningConstants.VISION_FAST_MOVING_PID_KF,
                    TuningConstants.VISION_FAST_MOVING_PID_KS,
                    TuningConstants.VISION_FAST_MOVING_PID_MIN,
                    TuningConstants.VISION_FAST_MOVING_PID_MAX,
                    this.getInjector().getInstance(ITimer.class));
                break;

            case Slow:
                this.movePIDHandler = new PIDHandler(
                    TuningConstants.VISION_SLOW_MOVING_PID_KP,
                    TuningConstants.VISION_SLOW_MOVING_PID_KI,
                    TuningConstants.VISION_SLOW_MOVING_PID_KD,
                    TuningConstants.VISION_SLOW_MOVING_PID_KF,
                    TuningConstants.VISION_SLOW_MOVING_PID_KS,
                    TuningConstants.VISION_SLOW_MOVING_PID_MIN,
                    TuningConstants.VISION_SLOW_MOVING_PID_MAX,
                    this.getInjector().getInstance(ITimer.class));
                break;

            default:
            case Normal:
                this.movePIDHandler = new PIDHandler(
                    TuningConstants.VISION_MOVING_PID_KP,
                    TuningConstants.VISION_MOVING_PID_KI,
                    TuningConstants.VISION_MOVING_PID_KD,
                    TuningConstants.VISION_MOVING_PID_KF,
                    TuningConstants.VISION_MOVING_PID_KS,
                    TuningConstants.VISION_MOVING_PID_MIN,
                    TuningConstants.VISION_MOVING_PID_MAX,
                    this.getInjector().getInstance(ITimer.class));
                break;
        }
    }

    @Override
    public void update()
    {
        super.update();

        Double currentValue = this.getMoveMeasuredValue();
        if (currentValue != null)
        {
            double desiredValue = this.getMoveDesiredValue(currentValue);
            double desiredVelocity = this.movePIDHandler.calculatePosition(desiredValue, currentValue);
            if (this.driveBackwards)
            {
                desiredVelocity *= -1.0;
            }

            switch (this.translateType)
            {
                case AprilTagStrafe:
                    this.setAnalogOperationState(AnalogOperation.DriveTrainMoveRight, desiredVelocity);
                    break;

                default:
                case AprilTagForward:
                    this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, -desiredVelocity);
                    break;
            }
        }
    }

    @Override
    public void end()
    {
        super.end();

        this.setDigitalOperationState(DigitalOperation.DriveTrainUseRobotOrientation, false);
        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveRight, 0.0);
    }

    @Override
    public boolean hasCompleted()
    {
        Double measuredValue = this.getMoveMeasuredValue();
        if (measuredValue == null)
        {
            return false;
        }

        double offset = Math.abs(this.getMoveDesiredValue(measuredValue) - measuredValue);

        // return false if we have not yet reached an acceptable offset from our goal position
        switch (this.translateType)
        {
            case AprilTagForward:
                if (offset > TuningConstants.MAX_VISION_ACCEPTABLE_FORWARD_DISTANCE)
                {
                    return false;
                }

                break;
            
            case AprilTagStrafe:
            default:
                if (offset > TuningConstants.MAX_VISION_ACCEPTABLE_STRAFE_DISTANCE)
                {
                    return false;
                }

                break;
        }

        return !this.verifyAngle || super.hasCompleted();
    }

    protected Double getMoveMeasuredValue()
    {
        if (this.translateType == MoveType.AprilTagForward)
        {
            return this.visionManager.getAprilTagXOffset();
        }
        else // if (this.translateType == MoveType.AprilTagStrafe)
        {
            return this.visionManager.getAprilTagYOffset();
        }
    }

    @Override
    protected PIDHandler createTurnHandler()
    {
        return new PIDHandler(
            TuningConstants.VISION_MOVING_TURNING_PID_KP,
            TuningConstants.VISION_MOVING_TURNING_PID_KI,
            TuningConstants.VISION_MOVING_TURNING_PID_KD,
            TuningConstants.VISION_MOVING_TURNING_PID_KF,
            TuningConstants.VISION_MOVING_TURNING_PID_KS,
            TuningConstants.VISION_MOVING_TURNING_PID_MIN,
            TuningConstants.VISION_MOVING_TURNING_PID_MAX,
            this.getInjector().getInstance(ITimer.class));
    }

    protected abstract double getMoveDesiredValue(double currentDistance);
}