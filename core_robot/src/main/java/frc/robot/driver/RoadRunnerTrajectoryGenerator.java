package frc.robot.driver;

import java.util.Arrays;

import com.acmerobotics.roadrunner.geometry.*;
import com.acmerobotics.roadrunner.trajectory.*;
import com.acmerobotics.roadrunner.trajectory.constraints.*;

import frc.lib.driver.TrajectoryManager;
import frc.lib.helpers.Helpers;
import frc.robot.HardwareConstants;
import frc.robot.TuningConstants;

public class RoadRunnerTrajectoryGenerator
{
    private static final TrajectoryVelocityConstraint velocityConstraint =
        new MinVelocityConstraint(
            Arrays.asList(
                new SwerveVelocityConstraint(
                    TuningConstants.SDSDRIVETRAIN_MAX_MODULE_PATH_VELOCITY,
                    HardwareConstants.SDSDRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE,
                    HardwareConstants.SDSDRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE),
                new AngularVelocityConstraint(TuningConstants.SDSDRIVETRAIN_MAX_PATH_TURN_VELOCITY * Helpers.DEGREES_TO_RADIANS),
                new TranslationalVelocityConstraint(TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY)));

    private static final TrajectoryAccelerationConstraint accelerationConstraint =
            new ProfileAccelerationConstraint(TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION);

    public static void main(String[] args)
    {
        TrajectoryManager trajectoryManager = new TrajectoryManager();
        RoadRunnerTrajectoryGenerator.generateTrajectories(trajectoryManager);
        trajectoryManager.buildAll();
    }

    public static void generateTrajectories(TrajectoryManager trajectoryManager)
    {
        // ----------------------------------------- Sample paths ----------------------------------------- //
        // addTrajectory(
        //     trajectoryManager,
        //     startTrajectory()
        //         .splineTo(new Vector2d(48, 0), 0),
        //     "goForward4ft");

        // addTrajectory(trajectoryManager, 
        //     startTrajectory(180.0 * Helpers.DEGREES_TO_RADIANS)
        //         .splineTo(new Vector2d(-4.0, 0), 180.0 * Helpers.DEGREES_TO_RADIANS), 
        //     "goBackwards4inch");

        // addTrajectory(
        //     trajectoryManager,
        //     startTrajectory(90.0 * Helpers.DEGREES_TO_RADIANS)
        //         .splineTo(new Vector2d(0, 48), 90.0 * Helpers.DEGREES_TO_RADIANS),
        //     "goLeft4ft");

        // addTrajectory(
        //     trajectoryManager,
        //     startTrajectory()
        //         .splineTo(new Vector2d(84, 0), 0),
        //     "goForward7ft");

        // addTrajectory(
        //     trajectoryManager,
        //     startTrajectory()
        //         .splineToSplineHeading(new Pose2d(-1, 0, 180.0 * Helpers.DEGREES_TO_RADIANS), 180.0 * Helpers.DEGREES_TO_RADIANS),
        //     "turn180Path");

        // addTrajectory(
        //     trajectoryManager,
        //     startTrajectory(180.0 * Helpers.DEGREES_TO_RADIANS)
        //         .splineToSplineHeading(new Pose2d(-84, 0, 180.0 * Helpers.DEGREES_TO_RADIANS), 180.0 * Helpers.DEGREES_TO_RADIANS),
        //     "goBack7ftRotate");

        // addTrajectory(
        //     trajectoryManager,
        //     startTrajectory(180.0 * Helpers.DEGREES_TO_RADIANS)
        //         .splineTo(new Vector2d(-72, 0), 180.0 * Helpers.DEGREES_TO_RADIANS),
        //     "goBack6ft");
    }

    private static TrajectoryBuilder startTrajectory()
    {
        return startTrajectory(0.0, 0.0, 0.0, 0.0);
    }

    private static TrajectoryBuilder startTrajectory(double startTangent)
    {
        return startTrajectory(0.0, 0.0, 0.0, startTangent);
    }

    private static TrajectoryBuilder startTrajectory(double startXPos, double startYPos, double startHeading, double startTangent)
    {
        return new TrajectoryBuilder(new Pose2d(startXPos, startYPos, startHeading), startTangent, RoadRunnerTrajectoryGenerator.velocityConstraint, RoadRunnerTrajectoryGenerator.accelerationConstraint);
    }

    //Ayush and Jamie constructor, takes Vector instead of X,Y doubles
    private static TrajectoryBuilder startTrajectory(Vector2d startVector, double startHeading, double startTangent)
    {
        return new TrajectoryBuilder(new Pose2d(startVector, startHeading), startTangent, RoadRunnerTrajectoryGenerator.velocityConstraint, RoadRunnerTrajectoryGenerator.accelerationConstraint);
    }

    private static void addTrajectory(TrajectoryManager trajectoryManager, TrajectoryBuilder trajectoryBuilder, String name)
    {
        try
        {
            trajectoryManager.addTrajectory(name, trajectoryBuilder);
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