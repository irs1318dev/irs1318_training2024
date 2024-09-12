package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class RumbleTask extends ControlTaskBase
{
    public RumbleTask()
    {
    }

    @Override
    public void begin()
    {
    }

    @Override
    public void update()
    {
        this.setDigitalOperationState(DigitalOperation.ForceLightDriverRumble, true);
    }

    @Override
    public void end()
    {
        this.setDigitalOperationState(DigitalOperation.ForceLightDriverRumble, false);
    }

    @Override
    public boolean hasCompleted()
    {
        return false;
    }
}
