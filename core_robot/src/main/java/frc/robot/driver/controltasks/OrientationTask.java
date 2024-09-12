package frc.robot.driver.controltasks;

import frc.lib.helpers.AnglePair;
import frc.robot.TuningConstants;
import frc.robot.driver.AnalogOperation;
import frc.robot.mechanisms.PigeonManager;

public class OrientationTask extends UpdateCycleTask
{
    private final double orientation;
    private final boolean waitUntilGoalReached;
    private final boolean keepOrienting;

    private PigeonManager pigeonManager;

    public OrientationTask(double orientation)
    {
        this(orientation, true);
    }

    public OrientationTask(double orientation, boolean waitUntilGoalReached)
    {
        this(orientation, waitUntilGoalReached, false);
    }

    public OrientationTask(double orientation, boolean waitUntilGoalReached, boolean keepOrienting)
    {
        super(1);

        this.orientation = orientation;
        this.waitUntilGoalReached = waitUntilGoalReached;
        this.keepOrienting = keepOrienting;
    }

    @Override
    public void begin()
    {
        super.begin();
        this.pigeonManager = this.getInjector().getInstance(PigeonManager.class);

        this.setAnalogOperationState(AnalogOperation.DriveTrainTurnAngleGoal, this.orientation);
    }

    @Override
    public void update()
    {
        super.update();
        this.setAnalogOperationState(AnalogOperation.DriveTrainTurnAngleGoal, this.orientation);
    }

    @Override
    public void end()
    {
        super.end();
        this.setAnalogOperationState(AnalogOperation.DriveTrainTurnAngleGoal, TuningConstants.MAGIC_NULL_VALUE);
    }

    @Override
    public boolean hasCompleted()
    {
        if (this.keepOrienting)
        {
            return false;
        }

        if (!this.waitUntilGoalReached)
        {
            return super.hasCompleted();
        }

        double currentYaw = this.pigeonManager.getYaw();
        double yawGoal = AnglePair.getClosestAngle(this.orientation, currentYaw, false).getAngle();
        // System.out.println("orient: " + currentYaw + "," + yawGoal);
        return Math.abs(currentYaw - yawGoal) < TuningConstants.ORIENTATION_TURN_THRESHOLD;
    }
}
