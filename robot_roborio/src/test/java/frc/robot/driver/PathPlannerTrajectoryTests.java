package frc.robot.driver;

import org.junit.jupiter.api.Test;

import frc.lib.driver.TrajectoryManager;
import frc.lib.robotprovider.PathPlannerWrapper;

public class PathPlannerTrajectoryTests
{
    @Test
    public void verifyTrajectoryGeneration()
    {
        TrajectoryManager trajectoryManager = new TrajectoryManager();
        PathPlannerTrajectoryGenerator.generateTrajectories(trajectoryManager, new PathPlannerWrapper());
        trajectoryManager.buildAll();
    }
}