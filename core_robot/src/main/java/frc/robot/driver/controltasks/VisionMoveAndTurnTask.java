package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class VisionMoveAndTurnTask extends VisionMoveAndTurnTaskBase
{
    private final double desiredDistance;

    /**
     * Initializes a new VisionMoveAndTurnTask
     * @param rotateType the type of rotation we are trying to perform (e.g. RetroReflective, AprilTag Center/Yaw, etc.)
     * @param translateType the type of translation we are trying to perform (e.g. Forward, Strafe)
     * @param moveSpeed which set of PID settings to use to move towards the goal position
     * @param bestEffort whether to consider the task successful/completed if we stop seeing the vision target instead of cancelled
     * @param verifyAngle whether to verify the angle when we declare completed as well as distance, or consider ourselves completed when we are at the goal distance
     * @param desiredDistance the desired distance value to reach
     */
    public VisionMoveAndTurnTask(TurnType rotateType, MoveType translateType, MoveSpeed moveSpeed, boolean bestEffort, boolean verifyAngle, double desiredDistance, DigitalOperation visionOperation)
    {
        this(rotateType, translateType, moveSpeed, bestEffort, verifyAngle, desiredDistance, visionOperation, false);
    }

    public VisionMoveAndTurnTask(TurnType rotateType, MoveType translateType, MoveSpeed moveSpeed, boolean bestEffort, boolean verifyAngle, double desiredDistance, DigitalOperation visionOperation, boolean backwards)
    {
        super(rotateType, translateType, moveSpeed, bestEffort, verifyAngle, visionOperation, backwards, false);
        this.desiredDistance = desiredDistance;
    }

    @Override
    protected double getMoveDesiredValue(double currentDistance)
    {
        return this.desiredDistance;
    }
}