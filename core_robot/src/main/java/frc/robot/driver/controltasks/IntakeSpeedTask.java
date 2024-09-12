package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.driver.*;
import frc.robot.mechanisms.*;

public class IntakeSpeedTask extends ControlTaskBase
{
    private enum IntakeState
    {
        DesiredGoal,
        Completed,
    }

    private final double intakeVelocity;
    private final boolean waitUntilPositionReached;

    private WristIntakeMechanism wrist;

    private IntakeState currentIntakeState;

    public IntakeSpeedTask(double intakeVelocity)
    {
        this(intakeVelocity, false);
    }

    public IntakeSpeedTask(double intakeVelocity, boolean waitUntilPositionReached)
    {
        this.intakeVelocity = intakeVelocity;
        this.waitUntilPositionReached = waitUntilPositionReached;
        this.currentIntakeState = IntakeState.DesiredGoal;
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
        if (this.currentIntakeState == IntakeState.DesiredGoal)
        {
            if (Math.abs(this.wrist.getVelocity() - this.intakeVelocity) < TuningConstants.INTAKE_MIN_THRESHOLD)
            {
                this.currentIntakeState = IntakeState.Completed;
            }
            else if (!this.waitUntilPositionReached)
            {
                this.currentIntakeState = IntakeState.Completed;
            }
        }

        switch (this.currentIntakeState)
        {
            default:
            case Completed:
            case DesiredGoal:
                this.setAnalogOperationState(AnalogOperation.IntakeMotorVelocityGoal, this.intakeVelocity);
                break;
        }
    }

    @Override
    public void end()
    {
        this.setAnalogOperationState(AnalogOperation.IntakeMotorVelocityGoal, TuningConstants.MAGIC_NULL_VALUE);
    }

    @Override
    public boolean hasCompleted()
    {
        return this.currentIntakeState == IntakeState.Completed;
    }
}
