package frc.lib.robotprovider;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import com.pathplanner.lib.path.*;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.helpers.ExceptionHelpers;
import frc.lib.helpers.Helpers;
import frc.lib.helpers.ImmutableQuadruple;

public class PathPlannerWrapper implements IPathPlanner
{
    public PathPlannerWrapper()
    {
    }

    @Override
    public ITrajectory loadTrajectory(String name, double maxVelocity, double maxAcceleration)
    {
        return this.loadTrajectory(name, maxVelocity, maxAcceleration, false);
    }

    @Override
    public ITrajectory loadTrajectory(String name, double maxVelocity, double maxAcceleration, boolean reversed)
    {
        PathPlannerPath path = PathPlannerPath.fromPathFile(name);
        return new PathPlannerTrajectoryWrapper(path.getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(0.0)));
    }

    @Override
    public ITrajectory buildTrajectory(
        double maxVelocity,
        double maxAcceleration,
        double maxAngularVelocity,
        double maxAngularAcceleration,
        IPathPlannerGoal... goalPoints)
    {
        PathConstraints constraints = new PathConstraints(
            maxVelocity * Helpers.METERS_PER_INCH,
            maxAcceleration * Helpers.METERS_PER_INCH,
            maxAngularVelocity * Helpers.DEGREES_TO_RADIANS,
            maxAngularAcceleration * Helpers.DEGREES_TO_RADIANS);

        ImmutableQuadruple<List<Pose2d>, List<RotationTarget>, Double, Double> quad = this.convertGoalPointsToPosesAndRotationTargets(goalPoints);
        if (quad == null)
        {
            return PathPlannerTrajectoryWrapper.Empty;
        }

        List<Translation2d> translations = PathPlannerPath.bezierFromPoses(quad.first);
        PathPlannerPath path =
            new PathPlannerPath(
                translations,
                quad.second,
                Collections.emptyList(),
                Collections.emptyList(),
                constraints,
                new GoalEndState(0.0, Rotation2d.fromDegrees(quad.fourth)),
                false);

        return new PathPlannerTrajectoryWrapper(
            new PathPlannerTrajectory(path, new ChassisSpeeds(), Rotation2d.fromDegrees(quad.third)));
    }

    private ImmutableQuadruple<List<Pose2d>, List<RotationTarget>, Double, Double> convertGoalPointsToPosesAndRotationTargets(IPathPlannerGoal[] goalPoints)
    {
        if (goalPoints == null || goalPoints.length < 2)
        {
            // note: this should already be verified in a unit test!!
            ExceptionHelpers.Assert(false, "there must be at least 2 goal points to make a trajectory!");
            return null;
        }

        ArrayList<Pose2d> poses = new ArrayList<Pose2d>(goalPoints.length);
        ArrayList<RotationTarget> rotations = new ArrayList<RotationTarget>(goalPoints.length);
        double firstRotation = 0.0;
        double lastRotation = 0.0;

        {
            IPathPlannerGoal firstGoalPoint = goalPoints[0];
            if (firstGoalPoint == null || !(firstGoalPoint instanceof PathPlannerWaypoint))
            {
                // note: this should already be verified in a unit test!!
                ExceptionHelpers.Assert(false, "The first goal point must be a PathPlannerWaypoint, actually: " + firstGoalPoint == null ? "null" : firstGoalPoint.getClass().getName());
                return null;
            }

            PathPlannerWaypoint firstWaypoint = (PathPlannerWaypoint)firstGoalPoint;
            if (!firstWaypoint.orientation.isPresent())
            {
                // note: this should already be verified in a unit test!!
                ExceptionHelpers.Assert(false, "The first goal point must have an orientation present");
                return null;
            }

            poses.add(new Pose2d(firstWaypoint.x * Helpers.METERS_PER_INCH, firstWaypoint.y * Helpers.METERS_PER_INCH, Rotation2d.fromDegrees(firstWaypoint.heading)));
            firstRotation = firstWaypoint.orientation.getAsDouble();
        }

        {
            int currentWaypointIdx = 0;
            for (int i = 1; i < goalPoints.length - 1; i++)
            {
                IPathPlannerGoal currentGoalPoint = goalPoints[i];
                if (currentGoalPoint instanceof PathPlannerWaypoint)
                {
                    PathPlannerWaypoint waypoint = (PathPlannerWaypoint)currentGoalPoint;
                    poses.add(new Pose2d(waypoint.x * Helpers.METERS_PER_INCH, waypoint.y * Helpers.METERS_PER_INCH, Rotation2d.fromDegrees(waypoint.heading)));
                    currentWaypointIdx++;
                    if (waypoint.orientation.isPresent())
                    {
                        // add a linked rotation target
                        rotations.add(new RotationTarget((double)currentWaypointIdx, Rotation2d.fromDegrees(waypoint.orientation.getAsDouble())));
                    }
                }
                else if (currentGoalPoint instanceof PathPlannerRotationTarget)
                {
                    PathPlannerRotationTarget rotation = (PathPlannerRotationTarget)currentGoalPoint;
                    rotations.add(new RotationTarget((double)currentWaypointIdx + rotation.percentage, Rotation2d.fromDegrees(rotation.orientation)));
                }
                else
                {
                    // note: this should already be verified in a unit test!!
                    ExceptionHelpers.Assert(false, "unknown type for goalPoint: " + currentGoalPoint == null ? "null" : currentGoalPoint.getClass().getName());
                }
            }
        }

        {
            IPathPlannerGoal lastGoalPoint = goalPoints[goalPoints.length - 1];
            if (lastGoalPoint == null || !(lastGoalPoint instanceof PathPlannerWaypoint))
            {
                // note: this should already be verified in a unit test!!
                ExceptionHelpers.Assert(false, "The last goal point must be a PathPlannerWaypoint, actually: " + lastGoalPoint == null ? "null" : lastGoalPoint.getClass().getName());
                return null;
            }

            PathPlannerWaypoint lastWaypoint = (PathPlannerWaypoint)lastGoalPoint;
            if (!lastWaypoint.orientation.isPresent())
            {
                // note: this should already be verified in a unit test!!
                ExceptionHelpers.Assert(false, "The last goal point must have an orientation present");
                return null;
            }

            poses.add(new Pose2d(lastWaypoint.x * Helpers.METERS_PER_INCH, lastWaypoint.y * Helpers.METERS_PER_INCH, Rotation2d.fromDegrees(lastWaypoint.heading)));
            lastRotation = lastWaypoint.orientation.getAsDouble();
        }

        return new ImmutableQuadruple<List<Pose2d>, List<RotationTarget>, Double, Double>(poses, rotations, firstRotation, lastRotation);
    }

    private static class PathPlannerTrajectoryWrapper implements ITrajectory
    {
        public static final PathPlannerTrajectoryWrapper Empty = new PathPlannerTrajectoryWrapper(null);

        private final PathPlannerTrajectory wrappedObject;

        PathPlannerTrajectoryWrapper(PathPlannerTrajectory wrappedObject)
        {
            this.wrappedObject = wrappedObject;
        }

        @Override
        public double getDuration()
        {
            if (this.wrappedObject == null)
            {
                return 0.0;
            }

            return this.wrappedObject.getTotalTimeSeconds();
        }

        @Override
        public TrajectoryState get(double time)
        {
            if (this.wrappedObject == null)
            {
                return null;
            }

            State state = (State)this.wrappedObject.sample(time);
            return new TrajectoryState(
                state.positionMeters.getX() * Helpers.INCHES_PER_METER,
                state.positionMeters.getY() * Helpers.INCHES_PER_METER,
                state.targetHolonomicRotation.getDegrees(),
                state.heading.getCos() * state.velocityMps * Helpers.INCHES_PER_METER,
                state.heading.getSin() * state.velocityMps * Helpers.INCHES_PER_METER,
                state.holonomicAngularVelocityRps.isPresent() ? state.holonomicAngularVelocityRps.get() * Helpers.RADIANS_TO_DEGREES : 0.0);
        }
    }
}
