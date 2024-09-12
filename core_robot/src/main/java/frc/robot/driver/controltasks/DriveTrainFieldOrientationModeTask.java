package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class DriveTrainFieldOrientationModeTask extends CompositeOperationTask
{
    private static final DigitalOperation[] possibleOperations =
        new DigitalOperation[]
        {
            DigitalOperation.DriveTrainEnableFieldOrientation,
            DigitalOperation.DriveTrainDisableFieldOrientation,
            DigitalOperation.DriveTrainUseRobotOrientation,
        };

    /**
     * Initializes a new FlywheelFixedSpinTask
     * @param enable
     */
    public DriveTrainFieldOrientationModeTask(boolean enable)
    {
        super(enable ? DigitalOperation.DriveTrainEnableFieldOrientation : DigitalOperation.DriveTrainDisableFieldOrientation, DriveTrainFieldOrientationModeTask.possibleOperations);
    }
}
