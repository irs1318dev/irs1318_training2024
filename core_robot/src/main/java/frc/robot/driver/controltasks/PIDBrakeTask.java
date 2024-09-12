package frc.robot.driver.controltasks;

import frc.robot.driver.*;
import frc.robot.mechanisms.IDriveTrainMechanism;
import frc.robot.mechanisms.SDSDriveTrainMechanism;

public class PIDBrakeTask extends ControlTaskBase
{
    private final boolean maintainPosition;

    private double[] steerSetpoints;
    private double[] driveSetpoints;

    public PIDBrakeTask()
    {
        this(false);
    }

    public PIDBrakeTask(boolean maintainPosition)
    {
        this.maintainPosition = maintainPosition;
    }

    /**
     * Begin the current task.
     */
    @Override
    public void begin()
    {
        IDriveTrainMechanism driveTrain = this.getInjector().getInstance(SDSDriveTrainMechanism.class);
        this.steerSetpoints = driveTrain.getModuleTurnInPlaceAngles();

        this.setDigitalOperationState(DigitalOperation.DriveTrainSteerMode, !this.maintainPosition);
        this.setDigitalOperationState(DigitalOperation.DriveTrainMaintainPositionMode, this.maintainPosition);
        this.setDigitalOperationState(DigitalOperation.DriveTrainPathMode, false);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPositionSteer1, this.steerSetpoints[0]);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPositionSteer2, this.steerSetpoints[1]);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPositionSteer3, this.steerSetpoints[2]);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPositionSteer4, this.steerSetpoints[3]);
        if (this.maintainPosition)
        {
            this.driveSetpoints = driveTrain.getDriveMotorPositions();
            this.setAnalogOperationState(AnalogOperation.DriveTrainPositionDrive1, this.driveSetpoints[0]);
            this.setAnalogOperationState(AnalogOperation.DriveTrainPositionDrive2, this.driveSetpoints[1]);
            this.setAnalogOperationState(AnalogOperation.DriveTrainPositionDrive3, this.driveSetpoints[2]);
            this.setAnalogOperationState(AnalogOperation.DriveTrainPositionDrive4, this.driveSetpoints[3]);
        }
    }

    /**
     * Run an iteration of the current task.
     */
    @Override
    public void update()
    {
        this.setDigitalOperationState(DigitalOperation.DriveTrainSteerMode, !this.maintainPosition);
        this.setDigitalOperationState(DigitalOperation.DriveTrainMaintainPositionMode, this.maintainPosition);
        this.setDigitalOperationState(DigitalOperation.DriveTrainPathMode, false);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPositionSteer1, this.steerSetpoints[0]);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPositionSteer2, this.steerSetpoints[1]);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPositionSteer3, this.steerSetpoints[2]);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPositionSteer4, this.steerSetpoints[3]);
        if (this.maintainPosition)
        {
            this.setAnalogOperationState(AnalogOperation.DriveTrainPositionDrive1, this.driveSetpoints[0]);
            this.setAnalogOperationState(AnalogOperation.DriveTrainPositionDrive2, this.driveSetpoints[1]);
            this.setAnalogOperationState(AnalogOperation.DriveTrainPositionDrive3, this.driveSetpoints[2]);
            this.setAnalogOperationState(AnalogOperation.DriveTrainPositionDrive4, this.driveSetpoints[3]);
        }
    }

    /**
     * Ends the current task, called when it (or a master task) has completed.
     */
    @Override
    public void end()
    {
        this.setDigitalOperationState(DigitalOperation.DriveTrainSteerMode, false);
        this.setDigitalOperationState(DigitalOperation.DriveTrainMaintainPositionMode, false);
        this.setDigitalOperationState(DigitalOperation.DriveTrainPathMode, false);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPositionDrive1, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPositionDrive2, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPositionDrive3, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPositionDrive4, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPositionSteer1, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPositionSteer2, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPositionSteer3, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPositionSteer4, 0.0);
    }

    /**
     * Checks whether this task has completed, or whether it should continue being processed.
     * @return true if we should continue onto the next task, otherwise false (to keep processing this task)
     */
    @Override
    public boolean hasCompleted()
    {
        return false;
    }
}
