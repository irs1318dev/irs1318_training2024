package frc.robot.mechanisms;

import frc.robot.ElectronicsConstants;
import frc.robot.LoggingKey;
import frc.lib.driver.IDriver;
import frc.lib.mechanisms.IMechanism;
import frc.lib.mechanisms.LoggingManager;
import frc.lib.robotprovider.ICompressor;
import frc.lib.robotprovider.ILogger;
import frc.lib.robotprovider.IRobotProvider;
import frc.lib.robotprovider.RobotMode;
import frc.robot.driver.DigitalOperation;

import com.google.inject.Inject;
import com.google.inject.Singleton;

/**
 * Compressor mechanism.
 * The mechanism defines the logic that controls a mechanism given inputs and operator-requested actions, and 
 * translates those into the abstract functions that should be applied to the outputs.
 * 
 * @author Will
 *
 */
@Singleton
public class CompressorMechanism implements IMechanism
{
    private final IDriver driver;
    private final ILogger logger;

    private final ICompressor compressor;

    private double preassureValue;

    private boolean isStarted;

    /**
     * Initializes a new CompressorMechanism
     * @param driver for obtaining operations
     * @param provider for obtaining electronics objects
     */
    @Inject
    public CompressorMechanism(IDriver driver, LoggingManager logger, IRobotProvider provider)
    {
        this.driver = driver;
        this.logger = logger;
        this.compressor = provider.getCompressor(ElectronicsConstants.PNEUMATICS_MODULE_A, ElectronicsConstants.PNEUMATICS_MODULE_TYPE_A);
        this.isStarted = false;
    }

    /**
     * read all of the sensors for the mechanism that we will use in macros/autonomous mode and record their values
     */
    @Override
    public void readSensors()
    {
        this.preassureValue = this.compressor.getPressure();
        this.logger.logNumber(LoggingKey.CompressorPreassure, this.preassureValue);
    }

    /**
     * calculate the various outputs to use based on the inputs and apply them to the outputs for the relevant mechanism
     */
    @Override
    public void update(RobotMode mode)
    {
        if (this.driver.getDigital(DigitalOperation.CompressorForceDisable))
        {
            if (this.isStarted)
            {
                this.compressor.disable();
                this.isStarted = false;
            }
        }
        else if (!this.isStarted)
        {
            if (ElectronicsConstants.PNEUMATICS_USE_HYBRID)
            {
                this.compressor.enableHybrid(ElectronicsConstants.PNEUMATICS_MIN_PSI, ElectronicsConstants.PNEUMATICS_MAX_PSI);
            }
            else if (ElectronicsConstants.PNEUMATICS_USE_ANALOG)
            {
                this.compressor.enableAnalog(ElectronicsConstants.PNEUMATICS_MIN_PSI, ElectronicsConstants.PNEUMATICS_MAX_PSI);
            }
            else
            {
                this.compressor.enableDigital();
            }

            this.isStarted = true;
        }
    }

    /**
     * stop the relevant mechanism
     */
    @Override
    public void stop()
    {
        this.compressor.disable();
        this.isStarted = false;
    }

    public double getPressureValue()
    {
        return this.preassureValue;
    }
}
