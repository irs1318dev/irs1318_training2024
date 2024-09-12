package frc.robot.driver.controltasks;

import frc.lib.driver.TrajectoryManager;
import frc.lib.helpers.ExceptionHelpers;
import frc.lib.helpers.Helpers;
import frc.lib.robotprovider.ITimer;
import frc.lib.robotprovider.ITrajectory;
import frc.lib.robotprovider.Pose2d;
import frc.lib.robotprovider.TrajectoryState;
import frc.robot.TuningConstants;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.IDriveTrainMechanism;
import frc.robot.mechanisms.RevDriveTrainMechanism;

/**
 * Task that follows a path
 * 
 */
public class FollowPathTask extends ControlTaskBase
{
    public enum Type
    {
        Absolute, // poses should match how they are on the field exactly
        RobotRelativeFromCurrentPose,
        FieldRelativeFromCurrentPose,
    }

    private final String pathName;
    private final Type type;

    private ITimer timer;

    private double startTime;
    private double trajectoryDuration;
    private ITrajectory trajectory;
    private Pose2d initialPose;

    /**
     * Initializes a new FollowPathTask
     * @param pathName the path to follow
     */
    public FollowPathTask(String pathName)
    {
        this(pathName, Type.RobotRelativeFromCurrentPose);
    }

    /**
     * Initializes a new FollowPathTask
     * @param pathName the path to follow
     * @param type describing how to follow the path
     */
    public FollowPathTask(String pathName, Type type)
    {
        this.pathName = pathName;
        this.type = type;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        this.timer = this.getInjector().getInstance(ITimer.class);

        TrajectoryManager trajectoryManager = this.getInjector().getInstance(TrajectoryManager.class);
        this.trajectory = trajectoryManager.getTrajectory(this.pathName);
        if (this.trajectory == null)
        {
            ExceptionHelpers.Assert(false, "Unknown trajectory '" + this.pathName + "'");
            this.startTime = 0.0;
            this.trajectoryDuration = 0.0;
            return;
        }

        this.startTime = this.timer.get();
        this.trajectoryDuration = this.trajectory.getDuration();

        if (this.type != Type.Absolute)
        {
            IDriveTrainMechanism driveTrain = this.getInjector().getInstance(RevDriveTrainMechanism.class);
            this.initialPose = driveTrain.getPose();
        }
        else
        {
            this.initialPose = new Pose2d(0.0, 0.0, 0.0);
        }

        this.setDigitalOperationState(DigitalOperation.DriveTrainPathMode, true);
    }

    /**
     * Run an iteration of the current task and apply any control changes
     */
    @Override
    public void update()
    { 
        TrajectoryState state = this.trajectory.get(this.timer.get() - this.startTime);

        double xPos = state.xPosition;
        double yPos = state.yPosition;
        double anglePos = state.angle;
        double xVel = state.xVelocity;
        double yVel = state.yVelocity;
        double angleVel = state.angleVelocity;

        double xGoal;
        double yGoal;
        double angleGoal;
        double xVelGoal;
        double yVelGoal;
        double angleVelGoal;
        switch (this.type)
        {
            case FieldRelativeFromCurrentPose:
                xGoal = xPos + this.initialPose.x;
                yGoal = yPos + this.initialPose.y;
                angleGoal = anglePos + this.initialPose.angle;
                xVelGoal = xVel;
                yVelGoal = yVel;
                angleVelGoal = angleVel;
                break;

            case RobotRelativeFromCurrentPose:
                double initialAngle = this.initialPose.angle;

                // change so that we move in relation to the direction the robot was initially pointing
                xGoal = Helpers.cosd(-initialAngle) * xPos + Helpers.sind(-initialAngle) * yPos + this.initialPose.x;
                yGoal = Helpers.cosd(-initialAngle) * yPos - Helpers.sind(-initialAngle) * xPos + this.initialPose.y;
                angleGoal = state.angle + initialAngle;
                xVelGoal = Helpers.cosd(-initialAngle) * xVel + Helpers.sind(-initialAngle) * yVel;
                yVelGoal = Helpers.cosd(-initialAngle) * yVel - Helpers.sind(-initialAngle) * xVel;
                angleVelGoal = state.angleVelocity;
                break;

            default:
            case Absolute:
                xGoal = xPos;
                yGoal = yPos;
                angleGoal = anglePos;
                xVelGoal = xVel;
                yVelGoal = yVel;
                angleVelGoal = angleVel;
                break;
        }

        this.setAnalogOperationState(AnalogOperation.DriveTrainPathXGoal, xGoal);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPathYGoal, yGoal);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPathAngleGoal, angleGoal);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPathXVelocityGoal, xVelGoal);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPathYVelocityGoal, yVelGoal);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPathAngleVelocityGoal, angleVelGoal);
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        this.setDigitalOperationState(DigitalOperation.DriveTrainPathMode, false);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPathXGoal, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPathYGoal, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPathAngleGoal, TuningConstants.MAGIC_NULL_VALUE);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPathXVelocityGoal, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPathYVelocityGoal, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainPathAngleVelocityGoal, 0.0);
    }

    /**
     * Checks whether this task has completed, or whether it should continue being processed
     * @return true if we should continue onto the next task, otherwise false (to keep processing this task)
     */
    @Override
    public boolean hasCompleted()
    {
        return this.timer.get() > this.startTime + this.trajectoryDuration;
    }
}
