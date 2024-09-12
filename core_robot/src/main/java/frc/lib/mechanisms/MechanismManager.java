package frc.lib.mechanisms;

import java.util.List;

import frc.lib.robotprovider.RobotMode;
import frc.robot.TuningConstants;

public class MechanismManager implements IMechanism
{
    public final List<IMechanism> mechanismList;

    public MechanismManager(List<IMechanism> mechanismList)
    {
        this.mechanismList = mechanismList;
    }

    @Override
    public void readSensors()
    {
        for (IMechanism mechanism : this.mechanismList)
        {
            try
            {
                mechanism.readSensors();
            }
            catch (Exception ex)
            {
                System.err.println("Encountered exception: " + ex.toString());
                if (TuningConstants.THROW_EXCEPTIONS)
                {
                    throw ex;
                }
            }
        }
    }

    @Override
    public void update(RobotMode mode)
    {
        for (IMechanism mechanism : this.mechanismList)
        {
            try
            {
                mechanism.update(mode);
            }
            catch (Exception ex)
            {
                System.err.println("Encountered exception: " + ex.toString());
                if (TuningConstants.THROW_EXCEPTIONS)
                {
                    throw ex;
                }
            }
        }
    }

    @Override
    public void stop()
    {
        for (IMechanism mechanism : this.mechanismList)
        {
            try
            {
                mechanism.stop();
            }
            catch (Exception ex)
            {
                System.err.println("Encountered exception: " + ex.toString());
                if (TuningConstants.THROW_EXCEPTIONS)
                {
                    throw ex;
                }
            }
        }
    }
}
