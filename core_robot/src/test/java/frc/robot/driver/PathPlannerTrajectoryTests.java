package frc.robot.driver;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import frc.lib.driver.TrajectoryManager;
import frc.lib.robotprovider.IPathPlanner;
import frc.lib.robotprovider.IPathPlannerGoal;
import frc.lib.robotprovider.ITrajectory;
import frc.lib.robotprovider.PathPlannerRotationTarget;
import frc.lib.robotprovider.PathPlannerWaypoint;

public class PathPlannerTrajectoryTests
{
    @Test
    public void verifyTrajectoryGenerationConstraints()
    {
        TrajectoryManager trajectoryManager = new TrajectoryManager();
        PathPlannerTrajectoryGenerator.generateTrajectories(
            trajectoryManager,
            new PathPlannerVerifier());
    }

    private class PathPlannerVerifier implements IPathPlanner
    {
        @Override
        public ITrajectory buildTrajectory(
            double maxVelocity,
            double maxAcceleration,
            double maxAngularVelocity,
            double maxAngularAcceleration,
            IPathPlannerGoal... goalPoints)
        {
            Assertions.assertTrue(maxVelocity > 0.0, "maxVelocity must be positive");
            Assertions.assertTrue(maxAcceleration > 0.0, "maxAcceleration must be positive");
            Assertions.assertTrue(maxAngularVelocity > 0.0, "maxAngularVelocity must be positive");
            Assertions.assertTrue(maxAngularAcceleration > 0.0, "maxAngularAcceleration must be positive");

            Assertions.assertNotNull(goalPoints, "goalPoints must be provided");
            Assertions.assertTrue(goalPoints.length >= 2, "at least 2 goalPoints must be provided");

            double lastPercentage = 0.0;
            for (int i = 0; i < goalPoints.length; i++)
            {
                IPathPlannerGoal goalPoint = goalPoints[i];
                Assertions.assertNotNull(goalPoint, "goal point at index " + i + " must be non-null");
                if (goalPoint instanceof PathPlannerRotationTarget)
                {
                    PathPlannerRotationTarget rotationTarget = (PathPlannerRotationTarget)goalPoint;
                    Assertions.assertTrue(rotationTarget.percentage > 0.0 && rotationTarget.percentage < 1.0, "rotation target percentage must be between 0 and 1, non-inclusive.  Actual: " + rotationTarget.percentage);
                    Assertions.assertTrue(rotationTarget.percentage >= lastPercentage, "rotation target percentage must be increasing");
                    lastPercentage = rotationTarget.percentage;
                }
                else if (goalPoint instanceof PathPlannerWaypoint)
                {
                    lastPercentage = 0.0;
                }
                else
                {
                    Assertions.fail("Unknown type for goalpoint " + i + ": " + goalPoint.getClass().getName());
                }
            }

            Assertions.assertInstanceOf(PathPlannerWaypoint.class, goalPoints[0], "the first goal point must be a PathPlannerWaypoint");
            PathPlannerWaypoint firstWaypoint = (PathPlannerWaypoint)goalPoints[0];
            Assertions.assertTrue(firstWaypoint.orientation.isPresent(), "the first waypoint must have an orientation");

            Assertions.assertInstanceOf(PathPlannerWaypoint.class, goalPoints[goalPoints.length - 1], "the last goal point must be a PathPlannerWaypoint");
            PathPlannerWaypoint lastWaypoint = (PathPlannerWaypoint)goalPoints[goalPoints.length - 1];
            Assertions.assertTrue(lastWaypoint.orientation.isPresent(), "the lasst waypoint must have an orientation");

            return null;
        }

        @Override
        public ITrajectory loadTrajectory(String name, double maxVelocity, double maxAcceleration)
        {
            return null;
        }

        @Override
        public ITrajectory loadTrajectory(String name, double maxVelocity, double maxAcceleration, boolean reversed)
        {
            return null;
        }
    }
}