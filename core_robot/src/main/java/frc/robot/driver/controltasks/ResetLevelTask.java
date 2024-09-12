package frc.robot.driver.controltasks;

import frc.robot.driver.*;

/**
 * Task that resets the robot's pitch/roll to be 0
 * 
 */
public class ResetLevelTask extends UpdateCycleTask
{
    /**
     * Initializes a new ResetLevelTask
     */
    public ResetLevelTask()
    {
        super(1);
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        super.begin();

        this.setDigitalOperationState(DigitalOperation.PositionResetRobotLevel, true);
    }

    /**
     * Run an iteration of the current task and apply any control changes 
     */
    @Override
    public void update()
    {
        super.update();

        this.setDigitalOperationState(DigitalOperation.PositionResetRobotLevel, true);
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        super.end();

        this.setDigitalOperationState(DigitalOperation.PositionResetRobotLevel, false);
    }
}
