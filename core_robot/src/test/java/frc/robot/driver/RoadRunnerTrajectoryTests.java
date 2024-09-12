package frc.robot.driver;

import org.junit.jupiter.api.Test;

import frc.lib.driver.TrajectoryManager;

public class RoadRunnerTrajectoryTests
{
    @Test
    public void verifyTrajectoryGeneration()
    {
        TrajectoryManager trajectoryManager = new TrajectoryManager();
        RoadRunnerTrajectoryGenerator.generateTrajectories(trajectoryManager);
        trajectoryManager.buildAll();
    }
}