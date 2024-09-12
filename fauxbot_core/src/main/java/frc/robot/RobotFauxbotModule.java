package frc.robot;

import frc.robot.simulation.*;

public class RobotFauxbotModule extends FauxbotModule
{
    @Override
    protected void configure()
    {
        super.configure();

        this.bind(SimulatorBase.class).to(RobotSimulator.class);
    }
}
