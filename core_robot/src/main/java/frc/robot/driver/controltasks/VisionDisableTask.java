package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class VisionDisableTask extends CompositeOperationTask
{
    private static final DigitalOperation[] possibleOperations =
        {
            DigitalOperation.VisionForceDisable,
        };

    public VisionDisableTask()
    {
        super(DigitalOperation.VisionForceDisable, VisionDisableTask.possibleOperations);
    }

    @Override
    public boolean hasCompleted()
    {
        return false;
    }
}
