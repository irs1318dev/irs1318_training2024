package frc.robot.mechanisms;

import frc.lib.driver.IDriver;
import frc.lib.mechanisms.LoggingManager;
import frc.lib.robotprovider.IPigeon2;
import frc.lib.robotprovider.ITalonFX;
import frc.lib.robotprovider.ITimer;
import frc.lib.robotprovider.JoystickRumbleType;
import frc.lib.robotprovider.MotorNeutralMode;
import frc.lib.robotprovider.NullLogger;
import frc.lib.robotprovider.Pose2d;
import frc.lib.robotprovider.RobotMode;
import frc.lib.robotprovider.TalonFXControlMode;
import frc.lib.robotprovider.TalonXLimitSwitchStatus;
import frc.lib.driver.descriptions.UserInputDevice;
import frc.lib.helpers.Helpers;
import frc.robot.HardwareConstants;
import frc.robot.TestProvider;
import frc.robot.TuningConstants;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class SDSDriveTrainMechanismTests
{
    private static final double[] MODULE_OFFSET_X =
        new double[]
        {
            -HardwareConstants.SDSDRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE, // module 1 (front-right)
            HardwareConstants.SDSDRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE, // module 2 (front-left)
            HardwareConstants.SDSDRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE, // module 3 (back-left)
            -HardwareConstants.SDSDRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE, // module 4 (back-right)
        };

        private static final double[] MODULE_OFFSET_Y =
        new double[]
        {
            -HardwareConstants.SDSDRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE, // module 1 (front-right)
            -HardwareConstants.SDSDRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE, // module 2 (front-left)
            HardwareConstants.SDSDRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE, // module 3 (back-left)
            HardwareConstants.SDSDRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE, // module 4 (back-right)
        };

    @Test
    public void testStill()
    {
        TestProvider provider = new TestProvider();

        MockTimer timer = new MockTimer();
        MockPigeon2 pigeon = new MockPigeon2();
        provider.setPigeon2(pigeon);

        MockTalonFX[] steer = new MockTalonFX[4];
        MockTalonFX[] drive = new MockTalonFX[4];
        for (int i = 0; i < 4; i++)
        {
            drive[i] = new MockTalonFX(2 * i + 1);
            steer[i] = new MockTalonFX(2 * i + 2);

            provider.setTalonFX(2 * i + 1, drive[i]);
            provider.setTalonFX(2 * i + 2, steer[i]);
        }

        IDriver driver = new MockDriver();
        LoggingManager logger = new LoggingManager(new NullLogger());
        PigeonManager pigeonManager = new PigeonManager(driver, logger, provider);
        PowerManager powerManager = new PowerManager(driver, timer, logger, provider);
        SDSDriveTrainMechanism driveTrain = new SDSDriveTrainMechanism(
            driver,
            logger,
            provider,
            pigeonManager,
            powerManager,
            timer);

        for (int timestep = 0; timestep < 10; timestep++)
        {
            pigeonManager.readSensors();
            driveTrain.readSensors();
            timer.increment(0.02);
        }

        Pose2d pose = driveTrain.getPose();
        assertEquals(0.0, pose.angle, 0.0001);
        assertEquals(0.0, pose.x, 0.0001);
        assertEquals(0.0, pose.y, 0.0001);
    }

    // @Test
    public void testForward1()
    {
        TestProvider provider = new TestProvider();

        MockTimer timer = new MockTimer();
        MockPigeon2 pigeon = new MockPigeon2();
        provider.setPigeon2(pigeon);

        MockTalonFX[] steer = new MockTalonFX[4];
        MockTalonFX[] drive = new MockTalonFX[4];
        for (int i = 0; i < 4; i++)
        {
            drive[i] = new MockTalonFX(2 * i + 1);
            steer[i] = new MockTalonFX(2 * i + 2);

            provider.setTalonFX(2 * i + 1, drive[i]);
            provider.setTalonFX(2 * i + 2, steer[i]);
        }

        IDriver driver = new MockDriver();
        LoggingManager logger = new LoggingManager(new NullLogger());
        PigeonManager pigeonManager = new PigeonManager(driver, logger, provider);
        PowerManager powerManager = new PowerManager(driver, timer, logger, provider);
        SDSDriveTrainMechanism driveTrain = new SDSDriveTrainMechanism(
            driver,
            logger,
            provider,
            pigeonManager,
            powerManager,
            timer);

        for (int timestep = 0; timestep < 50; timestep++)
        {
            for (int i = 0; i < 4; i++)
            {
                // 10% max-speed
                drive[i].set(0.1 * TuningConstants.SDSDRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KS);;
            }

            pigeonManager.readSensors();
            driveTrain.readSensors();
            timer.increment(0.02);
        }

        Pose2d pose = driveTrain.getPose();
        assertEquals(0.0, pose.angle, 0.5);
        assertEquals(0.0, pose.x, 0.5);
        assertEquals(0.1 * TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY, pose.y, 0.5);
    }

    // @Test
    public void testForward2()
    {
        TestProvider provider = new TestProvider();

        MockTimer timer = new MockTimer();
        MockPigeon2 pigeon = new MockPigeon2();
        provider.setPigeon2(pigeon);

        MockTalonFX[] steer = new MockTalonFX[4];
        MockTalonFX[] drive = new MockTalonFX[4];
        for (int i = 0; i < 4; i++)
        {
            drive[i] = new MockTalonFX(2 * i + 1);
            steer[i] = new MockTalonFX(2 * i + 2);

            provider.setTalonFX(2 * i + 1, drive[i]);
            provider.setTalonFX(2 * i + 2, steer[i]);
        }

        IDriver driver = new MockDriver();
        LoggingManager logger = new LoggingManager(new NullLogger());
        PigeonManager pigeonManager = new PigeonManager(driver, logger, provider);
        PowerManager powerManager = new PowerManager(driver, timer, logger, provider);
        SDSDriveTrainMechanism driveTrain = new SDSDriveTrainMechanism(
            driver,
            logger,
            provider,
            pigeonManager,
            powerManager,
            timer);

        for (int timestep = 0; timestep < 50; timestep++)
        {
            for (int i = 0; i < 4; i++)
            {
                // 10% max-speed
                drive[i].set(-0.1 * TuningConstants.SDSDRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KS);;
            }

            pigeon.set(180.0);
            pigeonManager.readSensors();
            driveTrain.readSensors();
            timer.increment(0.02);
        }

        Pose2d pose = driveTrain.getPose();
        assertEquals(180.0, pose.angle, 0.5);
        assertEquals(0.0, pose.x, 0.5);
        assertEquals(0.1 * TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY, pose.y, 0.5);
    }

    // @Test
    public void testLeft1()
    {
        TestProvider provider = new TestProvider();

        MockTimer timer = new MockTimer();
        MockPigeon2 pigeon = new MockPigeon2();
        provider.setPigeon2(pigeon);

        MockTalonFX[] steer = new MockTalonFX[4];
        MockTalonFX[] drive = new MockTalonFX[4];
        for (int i = 0; i < 4; i++)
        {
            drive[i] = new MockTalonFX(2 * i + 1);
            steer[i] = new MockTalonFX(2 * i + 2);

            provider.setTalonFX(2 * i + 1, drive[i]);
            provider.setTalonFX(2 * i + 2, steer[i]);
        }

        IDriver driver = new MockDriver();
        LoggingManager logger = new LoggingManager(new NullLogger());
        PigeonManager pigeonManager = new PigeonManager(driver, logger, provider);
        PowerManager powerManager = new PowerManager(driver, timer, logger, provider);
        SDSDriveTrainMechanism driveTrain = new SDSDriveTrainMechanism(
            driver,
            logger,
            provider,
            pigeonManager,
            powerManager,
            timer);

        for (int timestep = 0; timestep < 50; timestep++)
        {
            for (int i = 0; i < 4; i++)
            {
                // 10% max-speed
                drive[i].set(0.1 * TuningConstants.SDSDRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KS);;
                steer[i].set(90.0 * HardwareConstants.SDSDRIVETRAIN_STEER_TICKS_PER_DEGREE);
            }

            pigeon.set(0.0);
            pigeonManager.readSensors();
            driveTrain.readSensors();
            timer.increment(0.02);
        }

        Pose2d pose = driveTrain.getPose();
        assertEquals(0.0, pose.angle, 0.5);
        assertEquals(-0.1 * TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY, pose.x, 0.5);
        assertEquals(0.0, pose.y, 0.5);
    }

    // @Test
    public void testLeft2()
    {
        TestProvider provider = new TestProvider();

        MockTimer timer = new MockTimer();
        MockPigeon2 pigeon = new MockPigeon2();
        provider.setPigeon2(pigeon);

        MockTalonFX[] steer = new MockTalonFX[4];
        MockTalonFX[] drive = new MockTalonFX[4];
        for (int i = 0; i < 4; i++)
        {
            drive[i] = new MockTalonFX(2 * i + 1);
            steer[i] = new MockTalonFX(2 * i + 2);

            provider.setTalonFX(2 * i + 1, drive[i]);
            provider.setTalonFX(2 * i + 2, steer[i]);
        }

        IDriver driver = new MockDriver();
        LoggingManager logger = new LoggingManager(new NullLogger());
        PigeonManager pigeonManager = new PigeonManager(driver, logger, provider);
        PowerManager powerManager = new PowerManager(driver, timer, logger, provider);
        SDSDriveTrainMechanism driveTrain = new SDSDriveTrainMechanism(
            driver,
            logger,
            provider,
            pigeonManager,
            powerManager,
            timer);

        for (int timestep = 0; timestep < 50; timestep++)
        {
            for (int i = 0; i < 4; i++)
            {
                // 10% max-speed
                drive[i].set(0.1 * TuningConstants.SDSDRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KS);
            }

            pigeon.set(90.0);
            pigeonManager.readSensors();
            driveTrain.readSensors();
            timer.increment(0.02);
        }

        Pose2d pose = driveTrain.getPose();
        assertEquals(90.0, pose.angle, 0.5);
        assertEquals(-0.1 * TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY, pose.x, 0.5);
        assertEquals(0.0, pose.y, 0.5);
    }

    @Test
    public void testTwist1Rad()
    {
        TestProvider provider = new TestProvider();

        MockTimer timer = new MockTimer();
        MockPigeon2 pigeon = new MockPigeon2();
        provider.setPigeon2(pigeon);

        MockTalonFX[] steer = new MockTalonFX[4];
        MockTalonFX[] drive = new MockTalonFX[4];
        for (int i = 0; i < 4; i++)
        {
            steer[i] = new MockTalonFX(2 * i + 1);
            drive[i] = new MockTalonFX(2 * i + 2);

            provider.setTalonFX(2 * i + 1, steer[i]);
            provider.setTalonFX(2 * i + 2, drive[i]);
        }

        IDriver driver = new MockDriver();
        LoggingManager logger = new LoggingManager(new NullLogger());
        PigeonManager pigeonManager = new PigeonManager(driver, logger, provider);
        PowerManager powerManager = new PowerManager(driver, timer, logger, provider);
        SDSDriveTrainMechanism driveTrain = new SDSDriveTrainMechanism(
            driver,
            logger,
            provider,
            pigeonManager,
            powerManager,
            timer);

        double robotVelocityRight = 0.0;
        double robotVelocityForward = 0.0;
        double omega = 1.0;
        for (double timestep = 0; timestep <= 50.0; timestep += 1.0)
        {
            for (int i = 0; i < 4; i++)
            {
                double moduleVelocityRight = robotVelocityRight + omega * SDSDriveTrainMechanismTests.MODULE_OFFSET_Y[i];
                double moduleVelocityForward = robotVelocityForward - omega * SDSDriveTrainMechanismTests.MODULE_OFFSET_X[i];

                double moduleSteerPositionGoal = Helpers.atan2d(-moduleVelocityRight, moduleVelocityForward);
                moduleSteerPositionGoal *= TuningConstants.SDSDRIVETRAIN_STEER_MOTOR_POSITION_PID_KS;

                double moduleDriveVelocityGoal = Math.sqrt(moduleVelocityRight * moduleVelocityRight + moduleVelocityForward * moduleVelocityForward);
                moduleDriveVelocityGoal *= HardwareConstants.SDSDRIVETRAIN_DRIVE_INCHES_PER_SECOND_TO_MOTOR_VELOCITY;

                drive[i].set(moduleDriveVelocityGoal);
                steer[i].set(moduleSteerPositionGoal);
            }

            pigeon.set(timestep * 0.02 * 180.0 / Math.PI);
            pigeonManager.readSensors();
            driveTrain.readSensors();
            timer.increment(0.02);
        }

        Pose2d pose = driveTrain.getPose();
        assertEquals(180.0 / Math.PI, pose.angle, 1.0);
        assertEquals(0.0, pose.x, 0.5);
        assertEquals(0.0, pose.y, 0.5);
    }

    private class MockTimer implements ITimer
    {
        private double currentTime;

        MockTimer()
        {
            this.currentTime = 0.0;
        }

        @Override
        public void start()
        {
        }

        @Override
        public void stop()
        {
        }

        @Override
        public double get()
        {
            return this.currentTime;
        }

        @Override
        public void reset()
        {
            this.currentTime = 0.0;
        }

        // public void set(double value)
        // {
        //     this.currentTime = value;
        // }

        public void increment(double value)
        {
            this.currentTime += value;
        }
    }

    private class MockPigeon2 implements IPigeon2
    {
        private double currentAngle;

        MockPigeon2()
        {
        }

        public void getYawPitchRoll(double[] ypr_deg)
        {
            ypr_deg[0] = this.currentAngle;
            ypr_deg[1] = this.currentAngle;
            ypr_deg[2] = this.currentAngle;
        }

        public void getRollPitchYawRates(double[] xyz_dps)
        {
            xyz_dps[0] = this.currentAngle;
            xyz_dps[1] = this.currentAngle;
            xyz_dps[2] = this.currentAngle;
        }

        public void setYaw(double angleDeg)
        {
            this.currentAngle = angleDeg;
        }

        public void set(double angle)
        {
            this.currentAngle = angle;
        }

        @Override
        public void setYPRUpdateFrequency(double frequencyHz)
        {
        }

        @Override
        public void setRPYRateUpdateFrequency(double frequencyHz)
        {
        }
    }

    private class MockTalonFX implements ITalonFX
    {
        // private final int deviceId;
        private double currentValue;

        MockTalonFX(int deviceId)
        {
            // this.deviceId = deviceId;
        }

        @Override
        public void follow(ITalonFX talonFX)
        {
        }

        @Override
        public void setControlMode(TalonFXControlMode mode)
        {
        }

        @Override
        public void setSelectedSlot(int slotId)
        {
        }

        @Override
        public void setPIDF(double p, double i, double d, double f, int slotId)
        {
        }

        @Override
        public void setMotionMagicPIDVS(double p, double i, double d, double v, double s, double cruiseVelocity, double maxAcceleration, double maxJerk, int slotId)
        {
        }

        @Override
        public void setVoltageCompensation(boolean enabled, double maxVoltage)
        {
        }

        @Override
        public void stop()
        {
        }

        @Override
        public void setPosition(double position)
        {
        }

        @Override
        public void reset()
        {
        }

        @Override
        public double getPosition() 
        {
            return this.currentValue;
        }

        @Override
        public double getVelocity()
        {
            return this.currentValue;
        }

        @Override
        public double getError()
        {
            return 0;
        }

        @Override
        public boolean getForwardLimitSwitchClosed()
        {
            return false;
        }

        @Override
        public boolean getReverseLimitSwitchClosed()
        {
            return false;
        }

        @Override
        public TalonXLimitSwitchStatus getLimitSwitchStatus()
        {
            return null;
        }

        @Override
        public void set(double power)
        {
            this.currentValue = power;
        }

        @Override
        public void set(double value, double feedForward)
        {
            this.currentValue = value;
        }

        @Override
        public void set(TalonFXControlMode mode, double value)
        {
            this.currentValue = value;
        }

        @Override
        public void set(TalonFXControlMode mode, double value, double feedForward)
        {
            this.currentValue = value;
        }

        @Override
        public void set(TalonFXControlMode mode, int slotId, double value)
        {
            this.currentValue = value;
        }

        @Override
        public void set(TalonFXControlMode mode, int slotId, double value, double feedForward)
        {
            this.currentValue = value;
        }

        @Override
        public void setCurrentLimit(boolean enabled, double currentLimit, double triggerThresholdCurrent, double triggerThresholdTime)
        {
        }

        @Override
        public void setCurrentLimit(boolean enabled, double currentLimit, double triggerThresholdCurrent, double triggerThresholdTime, boolean statorLimiting, double statorCurrentLimit)
        {
        }

        @Override
        public void updateLimitSwitchConfig(boolean forwardEnabled, boolean forwardNormallyOpen, boolean reverseEnabled, boolean reverseNormallyOpen)
        {
        }

        @Override
        public void setMotorOutputSettings(boolean invert, MotorNeutralMode neutralMode)
        {
        }

        @Override
        public void follow(ITalonFX talonFX, boolean invertDirection)
        {
        }

        @Override
        public void clearRemoteSensor()
        {
        }

        @Override
        public void setRemoteSensor(int sensorId, double ratio)
        {
        }

        @Override
        public void setFeedbackUpdateRate(double frequencyHz)
        {
        }

        @Override
        public void setErrorUpdateRate(double frequencyHz)
        {
        }

        @Override
        public void setForwardLimitSwitchUpdateRate(double frequencyHz)
        {
        }

        @Override
        public void setReverseLimitSwitchUpdateRate(double frequencyHz)
        {
        }

        @Override
        public void optimizeCanbus()
        {
        }

        @Override
        public void updateLimitSwitchConfig(boolean forwardEnabled, boolean forwardNormallyOpen, boolean forwardReset, double forwardResetPosition, boolean reverseEnabled, boolean reverseNormallyOpen, boolean reverseReset, double reverseResetPosition)
        {
        }
    }

    private class MockDriver implements IDriver
    {
        private RobotMode currentMode = RobotMode.Teleop;

        @Override
        public RobotMode getMode()
        {
            return this.currentMode;
        }

        @Override
        public void update()
        {
        }

        @Override
        public void stop()
        {
        }

        @Override
        public void startMode(RobotMode mode)
        {
            this.currentMode = mode;
        }

        @Override
        public boolean getDigital(DigitalOperation digitalOperation)
        {
            return false;
        }

        @Override
        public double getAnalog(AnalogOperation analogOperation)
        {
            return 0.0;
        }

        @Override
        public void setRumble(UserInputDevice device, JoystickRumbleType type, double value)
        {
        }
    }
}
