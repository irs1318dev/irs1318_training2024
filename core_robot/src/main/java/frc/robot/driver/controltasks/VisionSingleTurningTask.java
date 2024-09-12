package frc.robot.driver.controltasks;

import frc.lib.helpers.Helpers;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.OffboardVisionManager;
import frc.robot.mechanisms.PigeonManager;

/**
 * Task that turns the robot a certain amount clockwise or counterclockwise in-place based on vision center
 */
public class VisionSingleTurningTask extends PIDTurnTaskBase
{
    public enum TurnType
    {
        None,
        AprilTagCentering,
        AprilTagParallelizing,
    }

    protected final TurnType rotateType;
    protected final DigitalOperation visionOperation;

    protected OffboardVisionManager visionManager;
    private PigeonManager pigeonManager;

    private Double ultimateYawAngle;

    /**
    * Initializes a new VisionSingleTurningTask
     * @param rotateType what type of turning to do
    */
    public VisionSingleTurningTask(TurnType rotateType, DigitalOperation visionOperation)
    {
        this(true, rotateType, false, visionOperation);
    }

    /**
    * Initializes a new VisionSingleTurningTask
     * @param rotateType what type of turning to do
     * @param bestEffort whether to end (true) or cancel (false, default) when we cannot see the game piece or vision target (for sequential tasks, whether to continue on or not)
    */
    public VisionSingleTurningTask(TurnType rotateType, boolean bestEffort, DigitalOperation visionOperation)
    {
        this(true, rotateType, bestEffort, visionOperation);
    }

    /**
     * Initializes a new VisionSingleTurningTask
     * @param useTime whether to make sure we are centered for a second or not
     * @param rotateType what type of turning to do
     * @param bestEffort whether to end (true) or cancel (false, default) when we cannot see the game piece or vision target (for sequential tasks, whether to continue on or not)
     */
    public VisionSingleTurningTask(boolean useTime, TurnType rotateType, boolean bestEffort, DigitalOperation visionOperation)
    {
        super(useTime, bestEffort);
        this.rotateType = rotateType;
        this.visionOperation = visionOperation;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        super.begin();

        this.visionManager = this.getInjector().getInstance(OffboardVisionManager.class);
        this.pigeonManager = this.getInjector().getInstance(PigeonManager.class);

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

    @Override
    protected Double getHorizontalAngle()
    {
        double pigeonYaw = this.pigeonManager.getYaw() + (this.pigeonManager.getAllianceSwapForward() ? 180.0 : 0.0);
        if (this.ultimateYawAngle == null)
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

            if (angle == null)
            {
                return null;
            }

            this.ultimateYawAngle = pigeonYaw + angle;
        }

        double result = this.ultimateYawAngle - pigeonYaw;
        return result;
    }
}