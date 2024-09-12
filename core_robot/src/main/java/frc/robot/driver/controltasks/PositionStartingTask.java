package frc.robot.driver.controltasks;

import frc.lib.helpers.ExceptionHelpers;
import frc.lib.robotprovider.Point2d;
import frc.robot.driver.*;

/**
 * Task that applies the starting angle
 * 
 */
public class PositionStartingTask extends UpdateCycleTask
{
    private final Double xPosition;
    private final Double yPosition;
    private final Double orientationAngle;
    private final boolean resetDriveTrain;
    private final boolean resetOrientation;

    /**
     * Initializes a new PositionStartingTask
     * @param xPosition - the offset to use for the x position from the field's "origin"
     * @param yPosition - the offset to use for the y position from the field's "origin"
     * @param orientationAngle - offset to use from the default of facing away from the alliance driver station (in degrees)
     */
    public PositionStartingTask(double xPosition, double yPosition, double orientationAngle)
    {
        this(xPosition, yPosition, orientationAngle, true, true);
    }

    /**
     * Initializes a new PositionStartingTask
     * @param xPosition - the offset to use for the x position from the field's "origin"
     * @param yPosition - the offset to use for the y position from the field's "origin"
     * @param orientationAngle - offset to use from the default of facing away from the alliance driver station (in degrees)
     * @param resetDriveTrain - whether to reset the drivetrain wheels (to read from the absolute encoders)
     * @param resetOrientation - whether to reset the orientation of the robot
     */
    public PositionStartingTask(Double xPosition, Double yPosition, Double orientationAngle, boolean resetDriveTrain, boolean resetOrientation)
    {
        super(1);

        ExceptionHelpers.Assert((xPosition == null) == (yPosition == null), "expect xPosition and yPosition to either both be null, or both be non-null");

        this.xPosition = xPosition;
        this.yPosition = yPosition;
        this.orientationAngle = orientationAngle;
        this.resetDriveTrain = resetDriveTrain;
        this.resetOrientation = resetOrientation;
    }

    /**
     * Initializes a new PositionStartingTask
     * @param xPosition - the offset to use for the x position from the field's "origin"
     * @param yPosition - the offset to use for the y position from the field's "origin"
     * @param orientationAngle - offset to use from the default of facing away from the alliance driver station (in degrees)
     * @param resetDriveTrain - whether to reset the drivetrain wheels (to read from the absolute encoders)
     * @param resetOrientation - whether to reset the orientation of the robot
     */
    public PositionStartingTask(Point2d point, Double orientationAngle, boolean resetDriveTrain, boolean resetOrientation)
    {
        super(1);

        this.xPosition = point.x;
        this.yPosition = point.y;
        this.orientationAngle = orientationAngle;
        this.resetDriveTrain = resetDriveTrain;
        this.resetOrientation = resetOrientation;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        super.begin();

        System.out.println("Hopefully this prints JAMIE 8:26 PM");

        this.setEverything();
    }

    /**
     * Run an iteration of the current task and apply any control changes 
     */
    @Override
    public void update()
    {
        super.update();

        this.setEverything();
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        super.end();

        if (this.orientationAngle != null)
        {
            this.setAnalogOperationState(AnalogOperation.PositionStartingAngle, 0.0);
        }

        if (this.xPosition != null || this.yPosition != null)
        {
            this.setDigitalOperationState(DigitalOperation.DriveTrainResetXYPosition, false);
            this.setAnalogOperationState(AnalogOperation.DriveTrainStartingXPosition, 0.0);
            this.setAnalogOperationState(AnalogOperation.DriveTrainStartingYPosition, 0.0);
        }

        this.setDigitalOperationState(DigitalOperation.DriveTrainReset, false);
        this.setDigitalOperationState(DigitalOperation.PositionResetFieldOrientation, false);
    }

    private void setEverything()
    {
        if (this.orientationAngle != null)
        {
            this.setAnalogOperationState(AnalogOperation.PositionStartingAngle, this.orientationAngle);
        }

        if (this.xPosition != null || this.yPosition != null)
        {
            this.setDigitalOperationState(DigitalOperation.DriveTrainResetXYPosition, true);
            this.setAnalogOperationState(AnalogOperation.DriveTrainStartingXPosition, this.xPosition);
            this.setAnalogOperationState(AnalogOperation.DriveTrainStartingYPosition, this.yPosition);
        }

        this.setDigitalOperationState(DigitalOperation.DriveTrainReset, this.resetDriveTrain);
        this.setDigitalOperationState(DigitalOperation.PositionResetFieldOrientation, this.resetOrientation);
    }
}
