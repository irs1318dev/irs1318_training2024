package frc.robot.driver.controltasks;

import java.util.Optional;

import frc.lib.helpers.AnglePair;
import frc.lib.robotprovider.Alliance;
import frc.lib.robotprovider.IDriverStation;
import frc.lib.robotprovider.IRobotProvider;
import frc.robot.TuningConstants;
import frc.robot.driver.AnalogOperation;
import frc.robot.mechanisms.PigeonManager;

public class FieldOrientationTask extends UpdateCycleTask
{
    public enum DesiredOrientation
    {
        Something,
    }

    private final boolean waitUntilGoalReached;
    private final boolean keepOrienting;
    private final DesiredOrientation orientation;

    private PigeonManager pigeonManager;
    private double desiredYaw;

    public FieldOrientationTask(DesiredOrientation orientation)
    {
        this(orientation, true);
    }

    public FieldOrientationTask(DesiredOrientation orientation, boolean waitUntilGoalReached)
    {
        this(orientation, waitUntilGoalReached, false);
    }

    public FieldOrientationTask(DesiredOrientation orientation, boolean waitUntilGoalReached, boolean keepOrienting)
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
        IRobotProvider robotProvider = this.getInjector().getInstance(IRobotProvider.class);
        IDriverStation driverStation = robotProvider.getDriverStation();
        Optional<Alliance> alliance = driverStation.getAlliance();

        boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
        switch (this.orientation)
        {
            case Something:
                this.desiredYaw = isRed ? -13.0 : 18.0;
                break;

            default:
                this.desiredYaw = 0.0;
                break;
        }

        this.setAnalogOperationState(AnalogOperation.DriveTrainTurnAngleGoal, this.desiredYaw);
    }

    @Override
    public void update()
    {
        super.update();
        this.setAnalogOperationState(AnalogOperation.DriveTrainTurnAngleGoal, this.desiredYaw);
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

        double currentYaw = this.pigeonManager.getYaw() + (this.pigeonManager.getAllianceSwapForward() ? 180.0 : 0.0);
        double yawGoal = AnglePair.getClosestAngle(this.desiredYaw, currentYaw, false).getAngle();
        return Math.abs(currentYaw - yawGoal) < TuningConstants.ORIENTATION_TURN_THRESHOLD;
    }
}
