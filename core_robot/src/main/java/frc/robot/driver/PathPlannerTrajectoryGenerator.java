package frc.robot.driver;

import frc.lib.driver.TrajectoryManager;
import frc.lib.robotprovider.IPathPlanner;
import frc.lib.robotprovider.ITrajectory;
import frc.lib.robotprovider.PathPlannerRotationTarget;
import frc.lib.robotprovider.PathPlannerWaypoint;
import frc.lib.robotprovider.Point2d;
import frc.robot.AutonLocManager;
import frc.robot.TuningConstants;

public class PathPlannerTrajectoryGenerator
{
    public static void generateTrajectories(TrajectoryManager trajectoryManager, IPathPlanner pathPlanner)
    {
        PathPlannerTrajectoryGenerator.generateTrajectories(false, trajectoryManager, pathPlanner);
        PathPlannerTrajectoryGenerator.generateTrajectories(true, trajectoryManager, pathPlanner);

        // ------------------------------- Example paths --------------------------------------------
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.REVDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 180.0, 0.0),
                new PathPlannerWaypoint(-15.0, 0.0, 180.0, 0.0),
                new PathPlannerWaypoint(-30.0, 0.0, 180.0, 0.0)),

            "goBackwards30in");
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.REVDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 180.0, 0.0),
                new PathPlannerWaypoint(-12.0, 0.0, 180.0, 0.0)),
            "goBackwards1ft");
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.REVDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 180.0, 0.0),
                new PathPlannerWaypoint(-6.0, 0.0, 180.0, 0.0),
                new PathPlannerWaypoint(-12.0, 0.0, 180.0, 0.0)),
            "goBackwards15in");
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.REVDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.REVDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 90.0, 0.0),
                new PathPlannerWaypoint(9.0, 16.0, 0.0, 0.0),
                new PathPlannerWaypoint(18.0, 32.0, 0.0, 0.0)),
            "goLeft32inForward18in");
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.REVDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.REVDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 270.0, 0.0),
                new PathPlannerWaypoint(9.0, -16.0, 0.0, 0.0),
                new PathPlannerWaypoint(18.0, -32.0, 0.0, 0.0)),
            "goRight32inForward18in");
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.REVDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.REVDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 90.0, 0.0),
                new PathPlannerWaypoint(0.0, 11.0, 90.0, 0.0),
                new PathPlannerWaypoint(0.0, 22.0, 90.0, 0.0)),
            "goLeft22in");
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.REVDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.REVDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 270.0, 0.0),
                new PathPlannerWaypoint(0.0, -22.0, 270.0, 0.0)),
            "goRight22in");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.REVDRIVETRAIN_LOW_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.REVDRIVETRAIN_LOW_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 0.0, 0.0),
                new PathPlannerWaypoint(0.0, 80.0, 0.0, 180.0),
                new PathPlannerWaypoint(0.0, -0.0, 0.0, 0.0),
                new PathPlannerWaypoint(0.0, 0.0, 0.0, 0.0)
                //new PathPlannerWaypoint(0.0, 0.0, 0.0, 0.0)
                ),
            "goJamieTask");


        // ------------------------------- Your paths go here --------------------------------------------
        
    }

    public static void generateTrajectories(boolean isRed, TrajectoryManager trajectoryManager, IPathPlanner pathPlanner)
    {
    }


    public static double getXPosition(boolean isRed, double position)
    {
        if(isRed)
        {
            return position;
        }
        else
        {
            return position * -1.0;
        }
    }

    //TODO can getOrientationorHeading() go in AutonLocManager?

    public static double getOrientationOrHeading(boolean isRed, double orientationOrHeading)
    {
        if(isRed)
        {
            return orientationOrHeading;
        }
        else
        {
            return 180.0 - orientationOrHeading;
        }
    }

    private static void addTrajectory(TrajectoryManager trajectoryManager, ITrajectory trajectory, String name)
    {
        // ExceptionHelpers.Assert(trajectory != null, "Adding null trajectory '%s'!", name);
        try
        {
            trajectoryManager.addTrajectory(name, trajectory);
        }
        catch (Exception ex)
        {
            System.err.println("Encountered exception generating path " + name + ": " + ex.toString());
            if (TuningConstants.THROW_EXCEPTIONS)
            {
                throw ex;
            }
        }
    }
}