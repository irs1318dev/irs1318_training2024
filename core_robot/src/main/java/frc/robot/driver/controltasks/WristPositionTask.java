package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.driver.*;
import frc.robot.mechanisms.*;

public class WristPositionTask extends ControlTaskBase
{
    private enum WristState
    {
        DesiredGoal,
        Completed,
    }

    private final double wristPosition;
    private final boolean waitUntilPositionReached;

    private WristIntakeMechanism wrist;

    private WristState currentWristState;

    public WristPositionTask(double wristPosition)
    {
        this(wristPosition, false);
    }

    public WristPositionTask(double wristPosition, boolean waitUntilPositionReached)
    {
        this.wristPosition = wristPosition;
        this.waitUntilPositionReached = waitUntilPositionReached;
        this.currentWristState = WristState.DesiredGoal;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        this.wrist = this.getInjector().getInstance(WristIntakeMechanism.class);
    }

    /**
     * Run an iteration of the current task and apply any control changes
     */
    @Override
    public void update()
    {
        if (this.currentWristState == WristState.DesiredGoal)
        {
            if (Math.abs(this.wrist.getPosition() - this.wristPosition) < TuningConstants.WRIST_MIN_THRESHOLD)
            {
                this.currentWristState = WristState.Completed;
            }
            else if (!this.waitUntilPositionReached)
            {
                this.currentWristState = WristState.Completed;
            }
        }

        switch (this.currentWristState)
        {
            default:
            case Completed:
            case DesiredGoal:
                this.setAnalogOperationState(AnalogOperation.WristSetAngle, this.wristPosition);
                break;
        }
    }

    @Override
    public void end()
    {
        this.setAnalogOperationState(AnalogOperation.WristSetAngle, TuningConstants.MAGIC_NULL_VALUE);
    }

    @Override
    public boolean hasCompleted()
    {
        return this.currentWristState == WristState.Completed;
    }
}
