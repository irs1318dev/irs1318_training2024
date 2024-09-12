package frc.lib.robotprovider;

import javax.inject.Singleton;

import frc.robot.TuningConstants;

@Singleton
public class RobotProvider implements IRobotProvider
{
    private IPowerDistribution powerDistribution;

    @Override
    public IAnalogInput getAnalogInput(int channel)
    {
        return new AnalogInputWrapper(channel);
    }

    @Override
    public IDigitalInput getDigitalInput(int channel)
    {
        return new DigitalInputWrapper(channel);
    }

    @Override
    public IDigitalOutput getDigitalOutput(int channel)
    {
        return new DigitalOutputWrapper(channel);
    }

    @Override
    public ICounter getCounter(int channel)
    {
        return new CounterWrapper(channel);
    }

    @Override
    public IDutyCycle getDutyCycle(int digitalInputChannel)
    {
        return new DutyCycleWrapper(digitalInputChannel);
    }

    @Override
    public IDutyCycleEncoder getDutyCycleEncoder(int digitalInputChannel)
    {
        return new DutyCycleEncoderWrapper(digitalInputChannel);
    }

    @Override
    public ITalonSRX getTalonSRX(int deviceNumber)
    {
        return new TalonSRXWrapper(deviceNumber);
    }

    @Override
    public ITalonFX getTalonFX(int deviceNumber)
    {
        return new TalonFXWrapper(deviceNumber);
    }

    @Override
    public ITalonFX getTalonFX(int deviceNumber, String canbus)
    {
        return new TalonFXWrapper(deviceNumber, canbus);
    }

    @Override
    public IVictorSPX getVictorSPX(int deviceNumber)
    {
        return new VictorSPXWrapper(deviceNumber);
    }

    @Override
    public ISparkMax getSparkMax(int deviceID, SparkMaxMotorType motorType)
    {
        return new SparkMaxWrapper(deviceID, motorType);
    }

    @Override
    public ICompressor getCompressor(PneumaticsModuleType moduleType)
    {
        return new CompressorWrapper(moduleType);
    }

    @Override
    public ICompressor getCompressor(int module, PneumaticsModuleType moduleType)
    {
        return new CompressorWrapper(module, moduleType);
    }

    @Override
    public IDoubleSolenoid getDoubleSolenoid(PneumaticsModuleType moduleType, int forwardChannel, int reverseChannel)
    {
        return new DoubleSolenoidWrapper(moduleType, forwardChannel, reverseChannel);
    }

    @Override
    public IDoubleSolenoid getDoubleSolenoid(int module, PneumaticsModuleType moduleType, int forwardChannel, int reverseChannel)
    {
        return new DoubleSolenoidWrapper(module, moduleType, forwardChannel, reverseChannel);
    }

    @Override
    public IEncoder getEncoder(int channelA, int channelB)
    {
        return new EncoderWrapper(channelA, channelB);
    }

    @Override
    public ICANCoder getCANCoder(int deviceNumber)
    {
        return new CANCoderWrapper(deviceNumber);
    }

    @Override
    public ICANCoder getCANCoder(int deviceNumber, String canbus)
    {
        return new CANCoderWrapper(deviceNumber, canbus);
    }

    @Override
    public IJoystick getJoystick(int port)
    {
        return new JoystickWrapper(port);
    }

    @Override
    public IMotor getTalon(int channel)
    {
        return new TalonWrapper(channel);
    }

    @Override
    public IMotor getVictor(int channel)
    {
        return new VictorWrapper(channel);
    }

    @Override
    public IServo getServo(int channel)
    {
        return new ServoWrapper(channel);
    }

    @Override
    public IPowerDistribution getPowerDistribution()
    {
        if (!TuningConstants.USE_ADVANTAGE_KIT || !TuningConstants.RETREIVE_PDH_FIRST)
        {
            return new PowerDistributionWrapper();
        }

        if (powerDistribution == null)
        {
            powerDistribution = new PowerDistributionWrapper();
        }

        return powerDistribution;
    }

    @Override
    public IPowerDistribution getPowerDistribution(int module, PowerDistributionModuleType moduleType)
    {
        if (!TuningConstants.USE_ADVANTAGE_KIT || !TuningConstants.RETREIVE_PDH_FIRST)
        {
            return new PowerDistributionWrapper(module, moduleType);
        }

        if (powerDistribution == null)
        {
            powerDistribution = new PowerDistributionWrapper(module, moduleType);
        }

        return powerDistribution;
    }

    @Override
    public IRelay getRelay(int channel)
    {
        return new RelayWrapper(channel);
    }

    @Override
    public IRelay getRelay(int channel, RelayDirection direction)
    {
        return new RelayWrapper(channel, direction);
    }

    @Override
    public ISolenoid getSolenoid(PneumaticsModuleType moduleType, int channel)
    {
        return new SolenoidWrapper(moduleType, channel);
    }

    @Override
    public ISolenoid getSolenoid(int module, PneumaticsModuleType moduleType, int channel)
    {
        return new SolenoidWrapper(module, moduleType, channel);
    }

    @Override
    public INavx getNavx()
    {
        return new NavxWrapper();
    }

    @Override
    public IPigeonIMU getPigeonIMU(int deviceNumber)
    {
        return new PigeonIMUWrapper(deviceNumber);
    }

    @Override
    public IPigeon2 getPigeon2(int deviceNumber)
    {
        return new Pigeon2Wrapper(deviceNumber);
    }

    @Override
    public IPigeon2 getPigeon2(int deviceNumber, String canbus)
    {
        return new Pigeon2Wrapper(deviceNumber, canbus);
    }

    @Override
    public ICANdle getCANdle(int deviceNumber)
    {
        return new CANdleWrapper(deviceNumber);
    }

    @Override
    public ICANdle getCANdle(int deviceNumber, String canbus)
    {
        return new CANdleWrapper(deviceNumber, canbus);
    }

    @Override
    public IDriverStation getDriverStation()
    {
        return DriverStationWrapper.Instance;
    }

    @Override
    public INetworkTableProvider getNetworkTableProvider()
    {
        if (TuningConstants.USE_ADVANTAGE_KIT)
        {
            return new AKNetworkTableProvider();
        }
        else
        {
            return new NetworkTableProvider();
        }
    }

    @Override
    public IPathPlanner getPathPlanner()
    {
        return new PathPlannerWrapper();
    }

    @Override
    public IPreferences getPreferences()
    {
        return new PreferencesWrapper();
    }
}
