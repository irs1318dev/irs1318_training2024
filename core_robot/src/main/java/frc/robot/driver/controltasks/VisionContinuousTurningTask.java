package frc.robot.driver.controltasks;

import frc.lib.controllers.PIDHandler;
import frc.lib.helpers.Helpers;
import frc.lib.robotprovider.ITimer;
import frc.robot.TuningConstants;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.OffboardVisionManager;

/**
 * Task that turns the robot a certain amount clockwise or counterclockwise in-place based on vision center
 */
public class VisionContinuousTurningTask extends PIDTurnTaskBase
{
    public enum TurnType
    {
        None,
        AprilTagCentering,
        AprilTagParallelizing,
    }

    protected final TurnType rotateType;
    protected final DigitalOperation visionOperation;
    protected final boolean continuous;

    protected OffboardVisionManager visionManager;

    /**
    * Initializes a new VisionContinuousTurningTask
     * @param rotateType what type of turning to do
     * @param visionOperation the type of vision to perform (front/rear, which apriltags, etc.)
    */
    public VisionContinuousTurningTask(TurnType rotateType, DigitalOperation visionOperation)
    {
        this(true, rotateType, false, visionOperation, false);
    }

    /**
    * Initializes a new VisionContinuousTurningTask
     * @param rotateType what type of turning to do
     * @param visionOperation the type of vision to perform (front/rear, which apriltags, etc.)
     * @param continuous whether to keep trying to center as long as we can see the vision target
    */
    public VisionContinuousTurningTask(TurnType rotateType, DigitalOperation visionOperation, boolean continuous)
    {
        this(true, rotateType, false, visionOperation, continuous);
    }

    /**
    * Initializes a new VisionContinuousTurningTask
     * @param rotateType what type of turning to do
     * @param bestEffort whether to end (true) or cancel (false, default) when we cannot see the game piece or vision target (for sequential tasks, whether to continue on or not)
     * @param visionOperation the type of vision to perform (front/rear, which apriltags, etc.)
    */
    public VisionContinuousTurningTask(TurnType rotateType, boolean bestEffort, DigitalOperation visionOperation)
    {
        this(true, rotateType, bestEffort, visionOperation, false);
    }

    /**
     * Initializes a new VisionContinuousTurningTask
     * @param useTime whether to make sure we are centered for a second or not
     * @param rotateType what type of turning to do
     * @param bestEffort whether to end (true) or cancel (false, default) when we cannot see the game piece or vision target (for sequential tasks, whether to continue on or not)
     * @param visionOperation the type of vision to perform (front/rear, which apriltags, etc.)
     * @param continuous whether to keep trying to center as long as we can see the vision target
     */
    public VisionContinuousTurningTask(boolean useTime, TurnType rotateType, boolean bestEffort, DigitalOperation visionOperation, boolean continuous)
    {
        super(useTime, bestEffort);
        this.rotateType = rotateType;
        this.visionOperation = visionOperation;
        this.continuous = continuous;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        super.begin();

        this.visionManager = this.getInjector().getInstance(OffboardVisionManager.class);

        for (DigitalOperation op : OffboardVisionManager.PossibleVisionOperations)
        {
            this.setDigitalOperationState(op, op == this.visionOperation);
        }
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        super.end();

        for (DigitalOperation op : OffboardVisionManager.PossibleVisionOperations)
        {
            this.setDigitalOperationState(op, false);
        }
    }

    /**
     * Checks whether this task has completed, or whether it should continue being processed
     * @return true if we should continue onto the next task, otherwise false (to keep processing this task)
     */
    @Override
    public boolean hasCompleted()
    {
        if (this.continuous)
        {
            return false;
        }

        return super.hasCompleted();
    }

    @Override
    protected PIDHandler createTurnHandler()
    {
        return new PIDHandler(
            TuningConstants.STATIONARY_CONTINUOUS_TURNING_PID_KP,
            TuningConstants.STATIONARY_CONTINUOUS_TURNING_PID_KI,
            TuningConstants.STATIONARY_CONTINUOUS_TURNING_PID_KD,
            TuningConstants.STATIONARY_CONTINUOUS_TURNING_PID_KF,
            TuningConstants.STATIONARY_CONTINUOUS_TURNING_PID_KS,
            TuningConstants.STATIONARY_CONTINUOUS_TURNING_PID_MIN,
            TuningConstants.STATIONARY_CONTINUOUS_TURNING_PID_MAX,
            this.getInjector().getInstance(ITimer.class));
    }

    @Override
    protected Double getHorizontalAngle()
    {
        Double angle;
        switch (this.rotateType)
        {
            case AprilTagParallelizing:
                // turn to match the yaw, so we are lined up parallel to the tag
                angle = this.visionManager.getAprilTagYaw();

                break;

            case AprilTagCentering:
                // Note: we want to point toward the AprilTag, not match its yaw (make ourselves parallel to it), so we can use the fact that tan(angle) = opposite / adjacent
                Double xOffset = this.visionManager.getAprilTagXOffset();
                Double yOffset = this.visionManager.getAprilTagYOffset();
                if (xOffset == null || yOffset == null)
                {
                    angle = null;
                }
                else
                {
                    angle = Helpers.atan2d(yOffset, xOffset);
                }

                break;

            default:
            case None:
                angle = 0.0;
                break;
        }

        return angle;
    }
}