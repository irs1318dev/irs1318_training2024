package frc.robot.mechanisms;

import frc.robot.*;
import frc.lib.driver.IDriver;
import frc.lib.filters.*;
import frc.lib.mechanisms.IMechanism;
import frc.lib.mechanisms.LoggingManager;
import frc.lib.robotprovider.*;

import com.google.inject.Inject;
import com.google.inject.Singleton;

/**
 * Power manager.
 * 
 * @author Will
 *
 */
@Singleton
public class PowerManager implements IMechanism
{
    public enum CurrentLimiting
    {
        Normal,
        OverCurrent,
        OverCurrentHigh,
    }

    private final IDriver driver;
    private final LoggingManager logger;
    private final IPowerDistribution powerDistribution;

    private final FadingMemoryFilter batteryVoltageFilter;

    private final FloatingAverageCalculator currentAverageCalculator;
    private double currentFloatingAverage;
    private double batteryVoltage;

    /**
     * Initializes a new PowerManager
     * @param provider for obtaining electronics objects
     */
    @Inject
    public PowerManager(IDriver driver, ITimer timer, LoggingManager logger, IRobotProvider provider)
    {
        this.driver = driver;
        this.logger = logger;
        this.powerDistribution = provider.getPowerDistribution(ElectronicsConstants.POWER_DISTRIBUTION_CAN_ID, ElectronicsConstants.POWER_DISTRIBUTION_TYPE);

        this.batteryVoltage = this.powerDistribution.getBatteryVoltage();
        this.batteryVoltageFilter = new FadingMemoryFilter(0.4, 0.6, this.batteryVoltage);

        this.currentAverageCalculator = new FloatingAverageCalculator(timer, TuningConstants.POWER_OVERCURRENT_TRACKING_MAX_VALUE, TuningConstants.POWER_OVERCURRENT_TRACKING_DURATION, TuningConstants.POWER_OVERCURRENT_SAMPLES_PER_SECOND);
        this.currentFloatingAverage = 0.0;
    }

    public CurrentLimiting getCurrentLimitingValue()
    {
        if (this.currentFloatingAverage <= TuningConstants.POWER_OVERCURRENT_THRESHOLD)
        {
            return CurrentLimiting.Normal;
        }
        else if (this.currentFloatingAverage <= TuningConstants.POWER_OVERCURREHT_HIGH_THRESHOLD)
        {
            return CurrentLimiting.OverCurrent;
        }
        else
        {
            return CurrentLimiting.OverCurrentHigh;
        }
    }

    public double getCurrent(int pdpChannel)
    {
        return this.powerDistribution.getCurrent(pdpChannel);
    }

    public double getBatteryVoltage()
    {
        return this.batteryVoltage;
    }

    @Override
    public void readSensors()
    {
        this.batteryVoltage = this.powerDistribution.getBatteryVoltage();
        this.logger.logNumber(LoggingKey.PowerBatteryVoltage, this.batteryVoltage);

        this.batteryVoltageFilter.update(this.batteryVoltage);
        this.logger.logNumber(LoggingKey.PowerBatteryVoltage, this.batteryVoltageFilter.getValue());

        double currCurrent = this.powerDistribution.getTotalCurrent();
        this.currentFloatingAverage = this.currentAverageCalculator.update(currCurrent);

        this.logger.logNumber(LoggingKey.PowerCurrent, currCurrent);
        this.logger.logNumber(LoggingKey.PowerCurrentFloatingAverage, this.currentFloatingAverage);
    }

    @Override
    public void update(RobotMode mode)
    {
    }

    @Override
    public void stop()
    {
        this.currentFloatingAverage = 0.0;
        this.currentAverageCalculator.reset();
        this.batteryVoltageFilter.reset();
    }
}
