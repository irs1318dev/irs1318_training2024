package frc.robot.mechanisms;

import frc.robot.*;
import frc.lib.controllers.*;
import frc.lib.driver.*;
import frc.lib.filters.*;
import frc.lib.helpers.AnglePair;
import frc.lib.helpers.Helpers;
import frc.lib.helpers.PoseHelpers;
import frc.lib.helpers.Triple;
import frc.lib.mechanisms.*;
import frc.lib.robotprovider.*;
import frc.robot.driver.*;
import frc.robot.mechanisms.PowerManager.CurrentLimiting;

import com.google.inject.Inject;
import com.google.inject.Singleton;

@Singleton
public class RevDriveTrainMechanism implements IDriveTrainMechanism
{
    private static final int NUM_MODULES = 4;

    private static final int DefaultPidSlotId = 0;
    private static final int AltPidSlotId = 1;

    private static final LoggingKey[] DRIVE_VELOCITY_LOGGING_KEYS = { LoggingKey.DriveTrainDriveVelocity1, LoggingKey.DriveTrainDriveVelocity2, LoggingKey.DriveTrainDriveVelocity3, LoggingKey.DriveTrainDriveVelocity4 };
    private static final LoggingKey[] DRIVE_POSITION_LOGGING_KEYS = { LoggingKey.DriveTrainDrivePosition1, LoggingKey.DriveTrainDrivePosition2, LoggingKey.DriveTrainDrivePosition3, LoggingKey.DriveTrainDrivePosition4 };
    private static final LoggingKey[] STEER_VELOCITY_LOGGING_KEYS = { LoggingKey.DriveTrainSteerVelocity1, LoggingKey.DriveTrainSteerVelocity2, LoggingKey.DriveTrainSteerVelocity3, LoggingKey.DriveTrainSteerVelocity4 };
    private static final LoggingKey[] STEER_ANGLE_LOGGING_KEYS = { LoggingKey.DriveTrainSteerAngle1, LoggingKey.DriveTrainSteerAngle2, LoggingKey.DriveTrainSteerAngle3, LoggingKey.DriveTrainSteerAngle4 };
    private static final LoggingKey[] DRIVE_GOAL_LOGGING_KEYS = { LoggingKey.DriveTrainDriveVelocityGoal1, LoggingKey.DriveTrainDriveVelocityGoal2, LoggingKey.DriveTrainDriveVelocityGoal3, LoggingKey.DriveTrainDriveVelocityGoal4 };
    private static final LoggingKey[] STEER_GOAL_LOGGING_KEYS = { LoggingKey.DriveTrainSteerPositionGoal1, LoggingKey.DriveTrainSteerPositionGoal2, LoggingKey.DriveTrainSteerPositionGoal3, LoggingKey.DriveTrainSteerPositionGoal4 };
    private static final LoggingKey[] STEER_GOAL_LOGGING_KEYS_B = { LoggingKey.DriveTrainSteerPositionGoal1b, LoggingKey.DriveTrainSteerPositionGoal2b, LoggingKey.DriveTrainSteerPositionGoal3b, LoggingKey.DriveTrainSteerPositionGoal4b };

    private static final AnalogOperation[] STEER_SETPOINT_OPERATIONS = new AnalogOperation[] { AnalogOperation.DriveTrainPositionSteer1, AnalogOperation.DriveTrainPositionSteer2, AnalogOperation.DriveTrainPositionSteer3, AnalogOperation.DriveTrainPositionSteer4 };
    private static final AnalogOperation[] DRIVE_SETPOINT_OPERATIONS = new AnalogOperation[] { AnalogOperation.DriveTrainPositionDrive1, AnalogOperation.DriveTrainPositionDrive2, AnalogOperation.DriveTrainPositionDrive3, AnalogOperation.DriveTrainPositionDrive4 };

    private static final double[] STEER_ABSOLUTE_OFFSETS = { TuningConstants.REVDRIVETRAIN_STEER_MOTOR1_ABSOLUTE_OFFSET, TuningConstants.REVDRIVETRAIN_STEER_MOTOR2_ABSOLUTE_OFFSET, TuningConstants.REVDRIVETRAIN_STEER_MOTOR3_ABSOLUTE_OFFSET, TuningConstants.REVDRIVETRAIN_STEER_MOTOR4_ABSOLUTE_OFFSET };

    // the x offsets of the swerve modules from the default center of rotation
    private final double[] moduleOffsetX;

    // the y offsets of the swerve modules from the default center of rotation
    private final double[] moduleOffsetY;

    private final IDriver driver;
    private final ILogger logger;
    private final ITimer timer;

    private final PigeonManager imuManager;
    private final PowerManager powerManager;

    private final ISparkMax[] steerMotors;
    private final ISparkMax[] driveMotors;

    private final PIDHandler omegaPID;
    private final boolean[] isDirectionSwapped;
    private final PIDHandler pathOmegaPID;
    private final PIDHandler pathXOffsetPID;
    private final PIDHandler pathYOffsetPID;
    private final int[] driveSlotIds;

    private final double[] driveVelocities;
    private final double[] drivePositions;
    private final double[] steerVelocities;
    private final double[] steerAngles;

    private final Triple<Double, Double, Double> driveTwistCorrection;
    private final Triple<Double, Double, Double> odometryTwistCorrection;
    private final Setpoint[] result;

    private final SlewRateLimiter xVelocityLimiter;
    private final SlewRateLimiter yVelocityLimiter;
    private final SlewRateLimiter angularVelocityLimiter;

    private final TrapezoidProfile[] steerTrapezoidMotionProfile;
    private final TrapezoidProfile.State[] steerTMPCurrState;
    private final TrapezoidProfile.State[] steerTMPGoalState;

    private boolean firstRun;

    private boolean fieldOriented;
    private boolean maintainOrientation;
    private double desiredYaw;

    private double time;

    // position, angle, and velocity of the robot based on odometry
    private double angle;
    private double xPosition;
    private double yPosition;
    private double forwardFieldVelocity;
    private double leftFieldVelocity;
    private double angularVelocity;

    private double deltaT;

    private double robotYaw;

    @Inject
    public RevDriveTrainMechanism(
        IDriver driver,
        LoggingManager logger,
        IRobotProvider provider,
        PigeonManager imuManager,
        PowerManager powerManager,
        ITimer timer)
    {
        this.driver = driver;
        this.logger = logger;
        this.timer = timer;
        this.imuManager = imuManager;
        this.powerManager = powerManager;

        this.steerMotors = new ISparkMax[RevDriveTrainMechanism.NUM_MODULES];
        this.driveMotors = new ISparkMax[RevDriveTrainMechanism.NUM_MODULES];

        this.moduleOffsetX =
            new double[]
            {
                -HardwareConstants.REVDRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE, // module 1 (front-right)
                HardwareConstants.REVDRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE, // module 2 (front-left)
                HardwareConstants.REVDRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE, // module 3 (back-left)
                -HardwareConstants.REVDRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE, // module 4 (back-right)
            };

        this.moduleOffsetY =
            new double[]
            {
                -HardwareConstants.REVDRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE, // module 1 (front-right)
                -HardwareConstants.REVDRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE, // module 2 (front-left)
                HardwareConstants.REVDRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE, // module 3 (back-left)
                HardwareConstants.REVDRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE, // module 4 (back-right)
            };

        int[] driveMotorCanIds =
            new int[]
            {
                ElectronicsConstants.REVDRIVETRAIN_DRIVE_MOTOR_1_CAN_ID,
                ElectronicsConstants.REVDRIVETRAIN_DRIVE_MOTOR_2_CAN_ID,
                ElectronicsConstants.REVDRIVETRAIN_DRIVE_MOTOR_3_CAN_ID,
                ElectronicsConstants.REVDRIVETRAIN_DRIVE_MOTOR_4_CAN_ID
            };

        int[] steerMotorCanIds =
            new int[]
            {
                ElectronicsConstants.REVDRIVETRAIN_STEER_MOTOR_1_CAN_ID,
                ElectronicsConstants.REVDRIVETRAIN_STEER_MOTOR_2_CAN_ID,
                ElectronicsConstants.REVDRIVETRAIN_STEER_MOTOR_3_CAN_ID,
                ElectronicsConstants.REVDRIVETRAIN_STEER_MOTOR_4_CAN_ID
            };

        boolean[] driveMotorInvertOutput =
            new boolean[]
            {
                HardwareConstants.REVDRIVETRAIN_DRIVE_MOTOR1_INVERT_OUTPUT,
                HardwareConstants.REVDRIVETRAIN_DRIVE_MOTOR2_INVERT_OUTPUT,
                HardwareConstants.REVDRIVETRAIN_DRIVE_MOTOR3_INVERT_OUTPUT,
                HardwareConstants.REVDRIVETRAIN_DRIVE_MOTOR4_INVERT_OUTPUT
            };

        boolean[] steerMotorInvertOutput =
            new boolean[]
            {
                HardwareConstants.REVDRIVETRAIN_STEER_MOTOR1_INVERT_OUTPUT,
                HardwareConstants.REVDRIVETRAIN_STEER_MOTOR2_INVERT_OUTPUT,
                HardwareConstants.REVDRIVETRAIN_STEER_MOTOR3_INVERT_OUTPUT,
                HardwareConstants.REVDRIVETRAIN_STEER_MOTOR4_INVERT_OUTPUT
            };

        boolean[] steerMotorInvertSensor =
            new boolean[]
            {
                HardwareConstants.REVDRIVETRAIN_STEER_MOTOR1_INVERT_SENSOR,
                HardwareConstants.REVDRIVETRAIN_STEER_MOTOR2_INVERT_SENSOR,
                HardwareConstants.REVDRIVETRAIN_STEER_MOTOR3_INVERT_SENSOR,
                HardwareConstants.REVDRIVETRAIN_STEER_MOTOR4_INVERT_SENSOR
            };

        if (TuningConstants.REVDRIVETRAIN_STEER_MOTORS_USE_TRAPEZOIDAL_MOTION_PROFILE)
        {
            this.steerTrapezoidMotionProfile = new TrapezoidProfile[RevDriveTrainMechanism.NUM_MODULES];
            this.steerTMPCurrState = new TrapezoidProfile.State[RevDriveTrainMechanism.NUM_MODULES];
            this.steerTMPGoalState = new TrapezoidProfile.State[RevDriveTrainMechanism.NUM_MODULES];
        }
        else
        {
            this.steerTrapezoidMotionProfile = null;
            this.steerTMPCurrState = null;
            this.steerTMPGoalState = null;
        }

        for (int i = 0; i < RevDriveTrainMechanism.NUM_MODULES; i++)
        {
            this.driveMotors[i] = provider.getSparkMax(driveMotorCanIds[i], SparkMaxMotorType.Brushless);
            this.driveMotors[i].setRelativeEncoder();
            // this.driveMotors[i].setInvertSensor(driveMotorInvertSensor[i]);
            this.driveMotors[i].setPositionConversionFactor(HardwareConstants.REVDRIVETRAIN_DRIVE_TICK_DISTANCE);
            this.driveMotors[i].setVelocityConversionFactor(HardwareConstants.REVDRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND);
            this.driveMotors[i].setNeutralMode(MotorNeutralMode.Brake);
            this.driveMotors[i].setInvertOutput(driveMotorInvertOutput[i]);
            this.driveMotors[i].setCurrentLimit(
                TuningConstants.REVDRIVETRAIN_DRIVE_CURRENT_STALL_LIMIT,
                TuningConstants.REVDRIVETRAIN_DRIVE_CURRENT_FREE_LIMIT,
                TuningConstants.REVDRIVETRAIN_DRIVE_CURRENT_LIMIT_RPM);
            this.driveMotors[i].setPIDF(
                TuningConstants.REVDRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KP, 
                TuningConstants.REVDRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KI,
                TuningConstants.REVDRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KD,
                TuningConstants.REVDRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KF,
                RevDriveTrainMechanism.DefaultPidSlotId);
            this.driveMotors[i].setPIDF(
                TuningConstants.REVDRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KP,
                TuningConstants.REVDRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KI,
                TuningConstants.REVDRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KD,
                TuningConstants.REVDRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KF,
                RevDriveTrainMechanism.AltPidSlotId);
            this.driveMotors[i].setControlMode(SparkMaxControlMode.Velocity);
            this.driveMotors[i].setSelectedSlot(RevDriveTrainMechanism.DefaultPidSlotId);
            this.driveMotors[i].burnFlash();

            this.steerMotors[i] = provider.getSparkMax(steerMotorCanIds[i], SparkMaxMotorType.Brushless);
            this.steerMotors[i].setAbsoluteEncoder();
            this.steerMotors[i].setInvertSensor(steerMotorInvertSensor[i]);
            this.steerMotors[i].setPositionConversionFactor(HardwareConstants.REVDRIVETRAIN_STEER_TICK_DISTANCE);
            this.steerMotors[i].setVelocityConversionFactor(HardwareConstants.REVDRIVETRAIN_STEER_MOTOR_VELOCITY_TO_DEGREES_PER_SECOND);
            // this.steerMotors[i].setAbsoluteOffset(RevDriveTrainMechanism.STEER_ABSOLUTE_OFFSETS[i]);
            this.steerMotors[i].setNeutralMode(MotorNeutralMode.Brake);
            this.steerMotors[i].setInvertOutput(steerMotorInvertOutput[i]);
            this.steerMotors[i].setCurrentLimit(
                TuningConstants.REVDRIVETRAIN_STEER_CURRENT_STALL_LIMIT,
                TuningConstants.REVDRIVETRAIN_STEER_CURRENT_FREE_LIMIT,
                TuningConstants.REVDRIVETRAIN_STEER_CURRENT_LIMIT_RPM);
            this.steerMotors[i].setPIDF(
                TuningConstants.REVDRIVETRAIN_STEER_MOTORS_POSITION_PID_KP,
                TuningConstants.REVDRIVETRAIN_STEER_MOTORS_POSITION_PID_KI,
                TuningConstants.REVDRIVETRAIN_STEER_MOTORS_POSITION_PID_KD,
                TuningConstants.REVDRIVETRAIN_STEER_MOTORS_POSITION_PID_KF,
                RevDriveTrainMechanism.DefaultPidSlotId);
            this.steerMotors[i].setPIDF(
                TuningConstants.REVDRIVETRAIN_STEER_MOTORS_TMP_PID_KP,
                TuningConstants.REVDRIVETRAIN_STEER_MOTORS_TMP_PID_KI,
                TuningConstants.REVDRIVETRAIN_STEER_MOTORS_TMP_PID_KD,
                TuningConstants.REVDRIVETRAIN_STEER_MOTORS_TMP_PID_KF,
                RevDriveTrainMechanism.AltPidSlotId);
            this.steerMotors[i].setPositionPIDWrappingSettings(
                TuningConstants.REVDRIVETRAIN_STEER_MOTORS_POSITION_PID_WRAPPING_ENABLED,
                TuningConstants.REVDRIVETRAIN_STEER_MOTORS_POSITION_PID_WRAPPING_MIN,
                TuningConstants.REVDRIVETRAIN_STEER_MOTORS_POSITION_PID_WRAPPING_MAX);
            this.steerMotors[i].setFeedbackFramePeriod(SparkMaxPeriodicFrameType.Status5, 10);
            this.steerMotors[i].setControlMode(SparkMaxControlMode.Position);

            if (TuningConstants.REVDRIVETRAIN_STEER_MOTORS_USE_TRAPEZOIDAL_MOTION_PROFILE)
            {
                this.steerTrapezoidMotionProfile[i] = new TrapezoidProfile(
                    TuningConstants.REVDRIVETRAIN_STEER_MOTORS_TMP_PID_CRUISE_VELOC,
                    TuningConstants.REVDRIVETRAIN_STEER_MOTORS_TMP_PID_ACCEL);
                this.steerTMPCurrState[i] = new TrapezoidProfile.State(0.0, 0.0);
                this.steerTMPGoalState[i] = new TrapezoidProfile.State(0.0, 0.0);

                this.steerMotors[i].setSelectedSlot(RevDriveTrainMechanism.AltPidSlotId);
            }
            else
            {
                this.steerMotors[i].setSelectedSlot(RevDriveTrainMechanism.DefaultPidSlotId);
            }
            
            this.steerMotors[i].burnFlash();
        }

        this.driveVelocities = new double[RevDriveTrainMechanism.NUM_MODULES];
        this.drivePositions = new double[RevDriveTrainMechanism.NUM_MODULES];
        this.steerVelocities = new double[RevDriveTrainMechanism.NUM_MODULES];
        this.steerAngles = new double[RevDriveTrainMechanism.NUM_MODULES];

        this.isDirectionSwapped = new boolean[RevDriveTrainMechanism.NUM_MODULES];
        this.driveSlotIds = new int[RevDriveTrainMechanism.NUM_MODULES];

        this.omegaPID = new PIDHandler(
            TuningConstants.REVDRIVETRAIN_OMEGA_POSITION_PID_KP,
            TuningConstants.REVDRIVETRAIN_OMEGA_POSITION_PID_KI,
            TuningConstants.REVDRIVETRAIN_OMEGA_POSITION_PID_KD,
            TuningConstants.REVDRIVETRAIN_OMEGA_POSITION_PID_KF,
            TuningConstants.REVDRIVETRAIN_OMEGA_POSITION_PID_KS,
            TuningConstants.REVDRIVETRAIN_OMEGA_MIN_OUTPUT,
            TuningConstants.REVDRIVETRAIN_OMEGA_MAX_OUTPUT,
            this.timer);

        this.pathOmegaPID = new PIDHandler(
            TuningConstants.REVDRIVETRAIN_PATH_OMEGA_POSITION_PID_KP,
            TuningConstants.REVDRIVETRAIN_PATH_OMEGA_POSITION_PID_KI,
            TuningConstants.REVDRIVETRAIN_PATH_OMEGA_POSITION_PID_KD,
            TuningConstants.REVDRIVETRAIN_PATH_OMEGA_POSITION_PID_KF,
            TuningConstants.REVDRIVETRAIN_PATH_OMEGA_POSITION_PID_KS,
            TuningConstants.REVDRIVETRAIN_PATH_OMEGA_MIN_OUTPUT,
            TuningConstants.REVDRIVETRAIN_PATH_OMEGA_MAX_OUTPUT,
            this.timer);

        this.pathXOffsetPID = new PIDHandler(
            TuningConstants.REVDRIVETRAIN_PATH_X_POSITION_PID_KP,
            TuningConstants.REVDRIVETRAIN_PATH_X_POSITION_PID_KI,
            TuningConstants.REVDRIVETRAIN_PATH_X_POSITION_PID_KD,
            TuningConstants.REVDRIVETRAIN_PATH_X_POSITION_PID_KF,
            TuningConstants.REVDRIVETRAIN_PATH_X_POSITION_PID_KS,
            TuningConstants.REVDRIVETRAIN_PATH_X_MIN_OUTPUT,
            TuningConstants.REVDRIVETRAIN_PATH_X_MAX_OUTPUT,
            this.timer);

        this.pathYOffsetPID = new PIDHandler(
            TuningConstants.REVDRIVETRAIN_PATH_Y_POSITION_PID_KP,
            TuningConstants.REVDRIVETRAIN_PATH_Y_POSITION_PID_KI,
            TuningConstants.REVDRIVETRAIN_PATH_Y_POSITION_PID_KD,
            TuningConstants.REVDRIVETRAIN_PATH_Y_POSITION_PID_KF,
            TuningConstants.REVDRIVETRAIN_PATH_Y_POSITION_PID_KS,
            TuningConstants.REVDRIVETRAIN_PATH_Y_MIN_OUTPUT,
            TuningConstants.REVDRIVETRAIN_PATH_Y_MAX_OUTPUT,
            this.timer);

        this.driveTwistCorrection = new Triple<Double, Double, Double>(0.0, 0.0, 0.0);
        this.odometryTwistCorrection = new Triple<Double, Double, Double>(0.0, 0.0, 0.0);
        this.result = new Setpoint[RevDriveTrainMechanism.NUM_MODULES];
        for (int i = 0; i < RevDriveTrainMechanism.NUM_MODULES; i++)
        {
            this.result[i] = new Setpoint();
        }

        if (TuningConstants.REVDRIVETRAIN_USE_TRANSLATIONAL_RATE_LIMITING)
        {
            this.xVelocityLimiter = new SlewRateLimiter(
                this.timer,
                TuningConstants.REVDRIVETRAIN_TRANSLATIONAL_VELOCITY_MAX_NEGATIVE_RATE,
                TuningConstants.REVDRIVETRAIN_TRANSLATIONAL_VELOCITY_MAX_POSITIVE_RATE,
                0.0);

            this.yVelocityLimiter = new SlewRateLimiter(
                this.timer,
                TuningConstants.REVDRIVETRAIN_TRANSLATIONAL_VELOCITY_MAX_NEGATIVE_RATE,
                TuningConstants.REVDRIVETRAIN_TRANSLATIONAL_VELOCITY_MAX_POSITIVE_RATE,
                0.0);
        }
        else
        {
            this.xVelocityLimiter = null;
            this.yVelocityLimiter = null;
        }

        if (TuningConstants.REVDRIVETRAIN_USE_ROTATIONAL_RATE_LIMITING)
        {
            this.angularVelocityLimiter = new SlewRateLimiter(
                this.timer,
                TuningConstants.REVDRIVETRAIN_ROTATIONAL_VELOCITY_MAX_NEGATIVE_RATE,
                TuningConstants.REVDRIVETRAIN_ROTATIONAL_VELOCITY_MAX_POSITIVE_RATE,
                0.0);
        }
        else
        {
            this.angularVelocityLimiter = null;
        }

        this.time = 0.0;
        this.angle = 0.0;
        this.xPosition = 0.0;
        this.yPosition = 0.0;

        this.firstRun = TuningConstants.REVDRIVETRAIN_RESET_ON_ROBOT_START;
        this.fieldOriented = TuningConstants.REVDRIVETRAIN_FIELD_ORIENTED_ON_ROBOT_START;
        this.maintainOrientation = TuningConstants.REVDRIVETRAIN_MAINTAIN_ORIENTATION_ON_ROBOT_START;
    }

    @Override
    public void readSensors()
    {
        for (int i = 0; i < RevDriveTrainMechanism.NUM_MODULES; i++)
        {
            this.driveVelocities[i] = this.driveMotors[i].getVelocity();
            this.drivePositions[i] = this.driveMotors[i].getPosition();
            this.steerVelocities[i] = this.steerMotors[i].getVelocity();
            this.steerAngles[i] = this.steerMotors[i].getPosition();

            this.logger.logNumber(RevDriveTrainMechanism.DRIVE_VELOCITY_LOGGING_KEYS[i], this.driveVelocities[i]);
            this.logger.logNumber(RevDriveTrainMechanism.DRIVE_POSITION_LOGGING_KEYS[i], this.drivePositions[i]);
            this.logger.logNumber(RevDriveTrainMechanism.STEER_VELOCITY_LOGGING_KEYS[i], this.steerVelocities[i]);
            this.logger.logNumber(RevDriveTrainMechanism.STEER_ANGLE_LOGGING_KEYS[i], this.steerAngles[i]);
        }

        double prevYaw = this.robotYaw;
        double prevTime = this.time;
        this.robotYaw = this.imuManager.getYaw();
        this.time = this.timer.get();

        this.deltaT = this.time - prevTime;
        if (this.deltaT <= 0.0)
        {
            this.deltaT = 0.001;
        }

        if (TuningConstants.REVDRIVETRAIN_USE_ODOMETRY)
        {
            double deltaImuYaw = (this.robotYaw - prevYaw) / this.deltaT;
            this.calculateOdometry(deltaImuYaw);
        }

        this.logger.logNumber(LoggingKey.DriveTrainXPosition, this.xPosition);
        this.logger.logNumber(LoggingKey.DriveTrainYPosition, this.yPosition);
        this.logger.logNumber(LoggingKey.DriveTrainAngle, this.angle);
    }

    @Override
    public void update(RobotMode mode)
    {
        if (this.driver.getDigital(DigitalOperation.DriveTrainEnableFieldOrientation))
        {
            this.fieldOriented = true;
            this.desiredYaw = this.robotYaw;
        }

        if (this.driver.getDigital(DigitalOperation.DriveTrainDisableFieldOrientation) ||
            !this.imuManager.getIsConnected())
        {
            this.fieldOriented = false;
        }

        boolean useFieldOriented = this.fieldOriented && !this.driver.getDigital(DigitalOperation.DriveTrainUseRobotOrientation);

        if (this.driver.getDigital(DigitalOperation.DriveTrainEnableMaintainDirectionMode))
        {
            this.maintainOrientation = true;
        }

        if (this.driver.getDigital(DigitalOperation.DriveTrainDisableMaintainDirectionMode) ||
            !this.imuManager.getIsConnected())
        {
            this.maintainOrientation = false;
        }

        boolean maintainOrientation = this.maintainOrientation;
        if (mode == RobotMode.Test)
        {
            maintainOrientation = false;
        }

        this.logger.logBoolean(LoggingKey.DriveTrainFieldOriented, useFieldOriented);
        this.logger.logBoolean(LoggingKey.DriveTrainMaintainOrientation, maintainOrientation);

        if (this.driver.getDigital(DigitalOperation.PositionResetFieldOrientation))
        {
            this.robotYaw = this.imuManager.getYaw();
            this.desiredYaw = this.robotYaw;
            this.angle = this.robotYaw;
        }

        if (this.driver.getDigital(DigitalOperation.DriveTrainResetXYPosition))
        {
            this.xPosition = this.driver.getAnalog(AnalogOperation.DriveTrainStartingXPosition);
            this.yPosition = this.driver.getAnalog(AnalogOperation.DriveTrainStartingYPosition);
        }

        double startingAngle = this.driver.getAnalog(AnalogOperation.PositionStartingAngle);
        if (startingAngle != TuningConstants.ZERO)
        {
            this.angle = startingAngle;
        }

        if (this.firstRun || this.driver.getDigital(DigitalOperation.DriveTrainReset))
        {
            for (int i = 0; i < RevDriveTrainMechanism.NUM_MODULES; i++)
            {
                this.driveMotors[i].setPosition(0);
                this.drivePositions[i] = 0.0;
                this.driveVelocities[i] = 0.0;
            }

            this.firstRun = false;
        }

        this.calculateSetpoints(useFieldOriented, maintainOrientation);
        for (int i = 0; i < RevDriveTrainMechanism.NUM_MODULES; i++)
        {
            Setpoint current = this.result[i];
            Double steerSetpoint = current.angle;
            Double driveVelocitySetpoint = current.driveVelocity;
            Double drivePositionSetpoint = current.drivePosition;

            SparkMaxControlMode driveControlMode = SparkMaxControlMode.PercentOutput; // Disabled doesn't exist for SparkMax
            int driveDesiredPidSlotId = RevDriveTrainMechanism.DefaultPidSlotId;
            double driveSetpoint = 0.0;
            if (driveVelocitySetpoint != null)
            {
                driveSetpoint = driveVelocitySetpoint;
                driveControlMode = SparkMaxControlMode.Velocity;
                driveDesiredPidSlotId = RevDriveTrainMechanism.DefaultPidSlotId;
            }
            else if (drivePositionSetpoint != null)
            {
                driveSetpoint = drivePositionSetpoint;
                driveControlMode = SparkMaxControlMode.Position;
                driveDesiredPidSlotId = RevDriveTrainMechanism.AltPidSlotId;
            }

            this.logger.logNumber(RevDriveTrainMechanism.DRIVE_GOAL_LOGGING_KEYS[i], driveSetpoint);
            this.driveMotors[i].setControlMode(driveControlMode);
            if (driveControlMode != SparkMaxControlMode.PercentOutput)
            {
                this.driveMotors[i].set(driveSetpoint);

                if (driveDesiredPidSlotId != this.driveSlotIds[i])
                {
                    this.driveMotors[i].setSelectedSlot(driveDesiredPidSlotId);
                    this.driveSlotIds[i] = driveDesiredPidSlotId;
                }
            }
            else
            {
                this.driveMotors[i].stop();
            }

            if (steerSetpoint != null ||
                TuningConstants.REVDRIVETRAIN_STEER_MOTORS_USE_TRAPEZOIDAL_MOTION_PROFILE)
            {
                this.logger.logNumber(RevDriveTrainMechanism.STEER_GOAL_LOGGING_KEYS[i], steerSetpoint);
                if (TuningConstants.REVDRIVETRAIN_STEER_MOTORS_USE_TRAPEZOIDAL_MOTION_PROFILE)
                {
                    TrapezoidProfile.State curr = this.steerTMPCurrState[i];
                    TrapezoidProfile.State goal = this.steerTMPGoalState[i];

                    if (steerSetpoint != null &&
                        goal.updatePosition((double)steerSetpoint))
                    {
                        // NOTE: we only update based on current positions if we are changing the goal
                        // otherwise we may fail to actually get the "oomph" to start moving because the
                        // initial acceleration could be very slow and lead to a very small velocity and
                        // very small position changes
                        // ALSO: we don't want to update the curr position directly to the steer angles because we want to
                        // make use of the SparkMAX's absolute positional PID mode.
                        double diff = steerSetpoint - this.steerAngles[i];
                        if (diff > 180.0)
                        {
                            // updating from low value to high value.  add 360 to the low value
                            curr.updatePosition(this.steerAngles[i] + 360.0);
                        }
                        else if (diff < -180.0)
                        {
                            // updating from high value to low value.  sub 360 from the high value
                            curr.updatePosition(this.steerAngles[i] - 360.0);
                        }
                        else
                        {
                            curr.updatePosition(this.steerAngles[i]);
                        }

                        // curr.setVelocity(this.steerVelocities[i]);
                    }

                    if (this.steerTrapezoidMotionProfile[i].update(this.deltaT, curr, goal))
                    {
                        steerSetpoint = Helpers.updateAngleRange360(curr.getPosition());
                    }
                }

                if (steerSetpoint != null)
                {
                    this.logger.logNumber(RevDriveTrainMechanism.STEER_GOAL_LOGGING_KEYS_B[i], steerSetpoint);
                    this.steerMotors[i].set(steerSetpoint);
                }
            }
        }
    }

    @Override
    public void stop()
    {
        this.omegaPID.reset();
        this.pathOmegaPID.reset();
        this.pathXOffsetPID.reset();
        this.pathYOffsetPID.reset();
        for (int i = 0; i < RevDriveTrainMechanism.NUM_MODULES; i++)
        {
            this.driveMotors[i].stop();
            this.steerMotors[i].stop();
        }

        if (this.xVelocityLimiter != null)
        {
            this.xVelocityLimiter.reset();
        }

        if (this.yVelocityLimiter != null)
        {
            this.yVelocityLimiter.reset();
        }

        if (this.angularVelocityLimiter != null)
        {
            this.angularVelocityLimiter.reset();
        }

        this.xPosition = 0.0;
        this.yPosition = 0.0;
    }

    public double[] getModuleTurnInPlaceAngles()
    {
        return new double[]
            {
                Helpers.atan2d(-this.moduleOffsetY[0], -this.moduleOffsetX[0]),
                Helpers.atan2d(-this.moduleOffsetY[1], -this.moduleOffsetX[1]),
                Helpers.atan2d(-this.moduleOffsetY[2], -this.moduleOffsetX[2]),
                Helpers.atan2d(-this.moduleOffsetY[3], -this.moduleOffsetX[3]),
            };
    }

    public double[] getDriveMotorPositions()
    {
        return new double[]
            {
                this.drivePositions[0],
                this.drivePositions[1],
                this.drivePositions[2],
                this.drivePositions[3],
            };
    }

    public Pose2d getPose()
    {
        return new Pose2d(this.xPosition, this.yPosition, this.robotYaw);
    }

    private void calculateSetpoints(boolean useFieldOriented, boolean maintainOrientation)
    {
        double xMult = 1.0;
        double yMult = 1.0;
        double yawAdj = 0.0;
        if (useFieldOriented && this.imuManager.getAllianceSwapForward())
        {
            xMult = -1.0;
            yMult = -1.0;
            yawAdj = 180.0;
        }

        boolean maintainPositionMode = this.driver.getDigital(DigitalOperation.DriveTrainMaintainPositionMode);
        if (maintainPositionMode || this.driver.getDigital(DigitalOperation.DriveTrainSteerMode))
        {
            for (int i = 0; i < RevDriveTrainMechanism.NUM_MODULES; i++)
            {
                this.result[i].driveVelocity = null;
                if (maintainPositionMode)
                {
                    this.result[i].drivePosition = this.driver.getAnalog(RevDriveTrainMechanism.DRIVE_SETPOINT_OPERATIONS[i]);
                }
                else
                {
                    this.result[i].drivePosition = null;
                }

                double moduleSteerPositionGoal = this.driver.getAnalog(RevDriveTrainMechanism.STEER_SETPOINT_OPERATIONS[i]);
                double currentAngle = this.steerAngles[i];
                AnglePair anglePair = AnglePair.getClosestAngleAbsolute(moduleSteerPositionGoal, currentAngle, this.isDirectionSwapped[i], true);
                moduleSteerPositionGoal = anglePair.getAngle();
                this.isDirectionSwapped[i] = anglePair.getSwapDirection();

                this.result[i].angle = moduleSteerPositionGoal;
            }

            return;
        }

        // calculate center velocity and turn velocity based on our current control mode:
        double rotationCenterA;
        double rotationCenterB;

        // robot center velocity, in inches/sec
        double centerVelocityForward;
        double centerVelocityLeft;

        // robot turn velocity, in rad/sec
        double omega;
        if (this.driver.getDigital(DigitalOperation.DriveTrainPathMode))
        {
            // path mode doesn't support rotation centers besides the robot center
            rotationCenterA = 0.0;
            rotationCenterB = 0.0;

            // Note: using the right-hand rule, "x" is forward, "y" is left, and "angle" is 0 straight ahead and increases counter-clockwise
            double xGoal = this.driver.getAnalog(AnalogOperation.DriveTrainPathXGoal);
            double yGoal = this.driver.getAnalog(AnalogOperation.DriveTrainPathYGoal);
            double angleGoal = this.driver.getAnalog(AnalogOperation.DriveTrainPathAngleGoal);
            double xVelocityGoal = this.driver.getAnalog(AnalogOperation.DriveTrainPathXVelocityGoal);
            double yVelocityGoal = this.driver.getAnalog(AnalogOperation.DriveTrainPathYVelocityGoal);
            double angleVelocityGoal = this.driver.getAnalog(AnalogOperation.DriveTrainPathAngleVelocityGoal);

            omega = angleVelocityGoal * Helpers.DEGREES_TO_RADIANS;
            if (useFieldOriented)
            {
                // add correction for x/y drift
                xVelocityGoal += this.pathXOffsetPID.calculatePosition(xGoal, this.xPosition);
                yVelocityGoal += this.pathYOffsetPID.calculatePosition(yGoal, this.yPosition);

                this.logger.logNumber(LoggingKey.DriveTrainXPositionGoal, xGoal);
                this.logger.logNumber(LoggingKey.DriveTrainYPositionGoal, yGoal);
                this.logger.logNumber(LoggingKey.DriveTrainAngleGoal, angleGoal);

                this.logger.logNumber(LoggingKey.DriveTrainXVelocityGoal, xVelocityGoal);
                this.logger.logNumber(LoggingKey.DriveTrainYVelocityGoal, yVelocityGoal);
                this.logger.logNumber(LoggingKey.DriveTrainAngleVelocityGoal, angleVelocityGoal);

                // convert velocity to be robot-oriented
                centerVelocityLeft = Helpers.cosd(this.robotYaw) * yVelocityGoal - Helpers.sind(this.robotYaw) * xVelocityGoal;
                centerVelocityForward = Helpers.cosd(this.robotYaw) * xVelocityGoal + Helpers.sind(this.robotYaw) * yVelocityGoal;

                // add correction for angle drift
                AnglePair anglePair = AnglePair.getClosestAngle(angleGoal, this.robotYaw, false);
                this.desiredYaw = anglePair.getAngle();

                this.logger.logNumber(LoggingKey.DriveTrainDesiredAngle, this.desiredYaw);
                omega += this.pathOmegaPID.calculatePosition(this.desiredYaw, this.robotYaw);
            }
            else
            {
                centerVelocityForward = xVelocityGoal;
                centerVelocityLeft = yVelocityGoal;
            }
        }
        else
        {
            // get the center of rotation modifying control values
            rotationCenterA = this.driver.getAnalog(AnalogOperation.DriveTrainRotationA);
            rotationCenterB = this.driver.getAnalog(AnalogOperation.DriveTrainRotationB);

            boolean useSlowMode = this.driver.getDigital(DigitalOperation.DriveTrainSlowMode);

            // get the center velocity control values (could be field-oriented or robot-oriented center velocity)
            double centerVelocityLeftRaw = -this.driver.getAnalog(AnalogOperation.DriveTrainMoveRight);
            double centerVelocityForwardRaw = this.driver.getAnalog(AnalogOperation.DriveTrainMoveForward);
            if (useSlowMode)
            {
                centerVelocityLeftRaw *= TuningConstants.REVDRIVETRAIN_SLOW_MODE_MAX_VELOCITY;
                centerVelocityForwardRaw *= TuningConstants.REVDRIVETRAIN_SLOW_MODE_MAX_VELOCITY;
            }
            else
            {
                centerVelocityLeftRaw *= TuningConstants.REVDRIVETRAIN_MAX_VELOCITY;
                centerVelocityForwardRaw *= TuningConstants.REVDRIVETRAIN_MAX_VELOCITY;
            }

            if (useFieldOriented)
            {
                centerVelocityLeft = yMult * (Helpers.cosd(this.robotYaw) * centerVelocityLeftRaw - Helpers.sind(this.robotYaw) * centerVelocityForwardRaw);
                centerVelocityForward = xMult * (Helpers.cosd(this.robotYaw) * centerVelocityForwardRaw + Helpers.sind(this.robotYaw) * centerVelocityLeftRaw);
            }
            else
            {
                centerVelocityLeft = centerVelocityLeftRaw;
                centerVelocityForward = centerVelocityForwardRaw;
            }

            boolean ignoreSlewRateLimiting = this.driver.getDigital(DigitalOperation.DriveTrainIgnoreSlewRateLimitingMode);
            if (this.xVelocityLimiter != null)
            {
                double rateLimitedCenterVelocityForward = this.xVelocityLimiter.update(centerVelocityForward);
                if (!ignoreSlewRateLimiting)
                {
                    centerVelocityForward = rateLimitedCenterVelocityForward;
                }
            }

            if (this.yVelocityLimiter != null)
            {
                double rateLimitedCenterVelocityLeft = this.yVelocityLimiter.update(centerVelocityLeft);
                if (!ignoreSlewRateLimiting)
                {
                    centerVelocityLeft = rateLimitedCenterVelocityLeft;
                }
            }

            omega = 0.0;
            double forcedOmega = this.driver.getAnalog(AnalogOperation.DriveTrainSpinLeft) + this.driver.getAnalog(AnalogOperation.DriveTrainSpinRight);
            if (forcedOmega != TuningConstants.ZERO)
            {
                this.desiredYaw = this.robotYaw;
                omega = forcedOmega;
                if (useSlowMode)
                {
                    omega *= TuningConstants.REVDRIVETRAIN_SLOW_MODE_TURN_SCALE;
                }
                else
                {
                    omega *= TuningConstants.REVDRIVETRAIN_TURN_SCALE;
                }

                if (this.angularVelocityLimiter != null)
                {
                    double rateLimitedAngularVelocity = this.angularVelocityLimiter.update(omega);
                    if (!ignoreSlewRateLimiting)
                    {
                        omega = rateLimitedAngularVelocity;
                    }
                }
            }
            else if (useFieldOriented)
            {
                boolean updatedOrientation = false;
                double yawGoal = this.driver.getAnalog(AnalogOperation.DriveTrainTurnAngleGoal);
                if (yawGoal != TuningConstants.MAGIC_NULL_VALUE)
                {
                    updatedOrientation = true;

                    AnglePair anglePair = AnglePair.getClosestAngle(yawGoal + yawAdj, this.robotYaw, false);
                    this.desiredYaw = anglePair.getAngle();
                }

                if (maintainOrientation || updatedOrientation)
                {
                    boolean skipTurn = false;
                    if (!updatedOrientation)
                    {
                        if (Math.abs(centerVelocityForward) + Math.abs(centerVelocityLeft) < TuningConstants.REVDRIVETRAIN_STATIONARY_VELOCITY)
                        {
                            skipTurn = TuningConstants.REVDRIVETRAIN_TURN_APPROXIMATION_STATIONARY != 0.0 && Helpers.WithinDelta(this.desiredYaw, this.robotYaw, TuningConstants.REVDRIVETRAIN_TURN_APPROXIMATION_STATIONARY);
                        }
                        else
                        {
                            skipTurn = TuningConstants.REVDRIVETRAIN_TURN_APPROXIMATION != 0.0 && Helpers.WithinDelta(this.desiredYaw, this.robotYaw, TuningConstants.REVDRIVETRAIN_TURN_APPROXIMATION);
                        }
                    }

                    // don't turn aggressively if we are within a very small delta from our goal angle
                    if (!skipTurn)
                    {
                        this.logger.logNumber(LoggingKey.DriveTrainDesiredAngle, this.desiredYaw);
                        omega = this.omegaPID.calculatePosition(this.desiredYaw, this.robotYaw);
                    }
                }
            }
        }

        if (TuningConstants.SDSDRIVETRAIN_USE_POSE_ESTIMATION_INVERSE_TWIST_CORRECTION)
        {
            this.driveTwistCorrection.set(
                centerVelocityForward * TuningConstants.SDSDRIVETRAIN_POSE_ESTIMATION_INVERSE_TWIST_CORRECTION_TIMESTEP,
                centerVelocityLeft * TuningConstants.SDSDRIVETRAIN_POSE_ESTIMATION_INVERSE_TWIST_CORRECTION_TIMESTEP,
                omega * TuningConstants.SDSDRIVETRAIN_POSE_ESTIMATION_INVERSE_TWIST_CORRECTION_TIMESTEP);

            PoseHelpers.inversePoseExponential(this.driveTwistCorrection);

            centerVelocityForward = this.driveTwistCorrection.getFirst() / TuningConstants.SDSDRIVETRAIN_POSE_ESTIMATION_INVERSE_TWIST_CORRECTION_TIMESTEP;
            centerVelocityLeft = this.driveTwistCorrection.getSecond() / TuningConstants.SDSDRIVETRAIN_POSE_ESTIMATION_INVERSE_TWIST_CORRECTION_TIMESTEP;
            omega = this.driveTwistCorrection.getThird() / TuningConstants.SDSDRIVETRAIN_POSE_ESTIMATION_INVERSE_TWIST_CORRECTION_TIMESTEP;
        }

        double maxModuleDriveVelocityGoal = 0.0;
        for (int i = 0; i < RevDriveTrainMechanism.NUM_MODULES; i++)
        {
            double moduleVelocityLeft = centerVelocityLeft - omega * (this.moduleOffsetY[i] + rotationCenterB);
            double moduleVelocityForward = centerVelocityForward - omega * (this.moduleOffsetX[i] + rotationCenterA);

            Double moduleSteerPositionGoal;
            double moduleDriveVelocityGoal;
            if (TuningConstants.REVDRIVETRAIN_SKIP_ANGLE_ON_ZERO_VELOCITY
                    && Helpers.WithinDelta(moduleVelocityLeft, 0.0, TuningConstants.REVDRIVETRAIN_SKIP_ANGLE_ON_ZERO_DELTA)
                    && Helpers.WithinDelta(moduleVelocityForward, 0.0, TuningConstants.REVDRIVETRAIN_SKIP_ANGLE_ON_ZERO_DELTA))
            {
                moduleDriveVelocityGoal = 0.0;
                moduleSteerPositionGoal = null;
            }
            else
            {
                moduleDriveVelocityGoal = Math.sqrt(moduleVelocityLeft * moduleVelocityLeft + moduleVelocityForward * moduleVelocityForward);

                moduleSteerPositionGoal = Helpers.atan2d(moduleVelocityLeft, moduleVelocityForward);
                double currentAngle = this.steerAngles[i];
                AnglePair anglePair = AnglePair.getClosestAngleAbsolute(moduleSteerPositionGoal, currentAngle, this.isDirectionSwapped[i], true);
                moduleSteerPositionGoal = anglePair.getAngle();
                this.isDirectionSwapped[i] = anglePair.getSwapDirection();

                if (maxModuleDriveVelocityGoal < moduleDriveVelocityGoal)
                {
                    maxModuleDriveVelocityGoal = moduleDriveVelocityGoal;
                }

                if (this.isDirectionSwapped[i])
                {
                    moduleDriveVelocityGoal *= -1.0;
                }
            }

            this.result[i].driveVelocity = moduleDriveVelocityGoal;
            this.result[i].drivePosition = null;
            this.result[i].angle = moduleSteerPositionGoal;
        }

        // rescale velocities based on max velocity percentage, if max velocity is exceeded for any module
        if (maxModuleDriveVelocityGoal > TuningConstants.REVDRIVETRAIN_MAX_VELOCITY)
        {
            // divide by percentage is interchangeable with multiply by inverse-percentage
            double invPercentage = TuningConstants.REVDRIVETRAIN_MAX_VELOCITY / maxModuleDriveVelocityGoal;
            for (int i = 0; i < RevDriveTrainMechanism.NUM_MODULES; i++)
            {
                this.result[i].driveVelocity *= invPercentage;
            }
        }

        if (TuningConstants.REVDRIVETRAIN_USE_OVERCURRENT_ADJUSTMENT)
        {
            CurrentLimiting value = this.powerManager.getCurrentLimitingValue();
            if (value != CurrentLimiting.Normal)
            {
                double currentLimitingMultiplier;
                if (value == CurrentLimiting.OverCurrent)
                {
                    currentLimitingMultiplier = TuningConstants.REVDRIVETRAIN_OVERCURRENT_ADJUSTMENT;
                }
                else // if (value == CurrentLimiting.OverCurrentHigh)
                {
                    currentLimitingMultiplier = TuningConstants.REVDRIVETRAIN_OVERCURRENT_HIGH_ADJUSTMENT;
                }

                for (int i = 0; i < RevDriveTrainMechanism.NUM_MODULES; i++)
                {
                    this.result[i].driveVelocity *= currentLimitingMultiplier;
                }
            }
        }
    }

    private void calculateOdometry(double deltaImuYaw)
    {
        // double imuOmega = deltaImuYaw / this.deltaT; // in degrees
        double leftRobotVelocity;
        double forwardRobotVelocity;

        // calculate our right and forward velocities using an average of our various velocities and the angle.
        double rightRobotVelocity1 = -Helpers.sind(this.steerAngles[0]) * this.driveVelocities[0];
        double rightRobotVelocity2 = -Helpers.sind(this.steerAngles[1]) * this.driveVelocities[1];
        double rightRobotVelocity3 = -Helpers.sind(this.steerAngles[2]) * this.driveVelocities[2];
        double rightRobotVelocity4 = -Helpers.sind(this.steerAngles[3]) * this.driveVelocities[3];

        double forwardRobotVelocity1 = Helpers.cosd(this.steerAngles[0]) * this.driveVelocities[0];
        double forwardRobotVelocity2 = Helpers.cosd(this.steerAngles[1]) * this.driveVelocities[1];
        double forwardRobotVelocity3 = Helpers.cosd(this.steerAngles[2]) * this.driveVelocities[2];
        double forwardRobotVelocity4 = Helpers.cosd(this.steerAngles[3]) * this.driveVelocities[3];

        // rightRobotVelocity = (rightRobotVelocity1 + rightRobotVelocity2 + rightRobotVelocity3 + rightRobotVelocity4) / 4.0;
        // forwardRobotVelocity = (forwardRobotVelocity1 + forwardRobotVelocity2 + forwardRobotVelocity3 + forwardRobotVelocity4) / 4.0;

        double a = -0.5 * (rightRobotVelocity3 + rightRobotVelocity4);
        double b = -0.5 * (rightRobotVelocity1 + rightRobotVelocity2);
        double c = 0.5 * (forwardRobotVelocity1 + forwardRobotVelocity4);
        double d = 0.5 * (forwardRobotVelocity2 + forwardRobotVelocity3);

        double omegaRadians1 = (b - a) * HardwareConstants.REVDRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE_INV;
        double omegaRadians2 = (c - d) * HardwareConstants.REVDRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE_INV;
        double omegaRadians = 0.5 * (omegaRadians1 + omegaRadians2);

        double rightRobotVelocityA = omegaRadians * HardwareConstants.REVDRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE + a;
        double rightRobotVelocityB = -omegaRadians * HardwareConstants.REVDRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE + b;
        leftRobotVelocity = 0.5 * (rightRobotVelocityA + rightRobotVelocityB);

        double forwardRobotVelocityA = omegaRadians * HardwareConstants.REVDRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE + c;
        double forwardRobotVelocityB = -omegaRadians * HardwareConstants.REVDRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE + d;
        forwardRobotVelocity = 0.5 * (forwardRobotVelocityA + forwardRobotVelocityB);

        double leftFieldVelocity = leftRobotVelocity * Helpers.cosd(this.robotYaw) + forwardRobotVelocity * Helpers.sind(this.robotYaw);
        double forwardFieldVelocity = -leftRobotVelocity * Helpers.sind(this.robotYaw) + forwardRobotVelocity * Helpers.cosd(this.robotYaw);

        double omegaDegrees = omegaRadians * Helpers.RADIANS_TO_DEGREES;
        if (TuningConstants.SDSDRIVETRAIN_USE_POSE_ESTIMATION_INVERSE_TWIST_CORRECTION)
        {
            this.odometryTwistCorrection.set(
                forwardFieldVelocity * TuningConstants.SDSDRIVETRAIN_POSE_ESTIMATION_INVERSE_TWIST_CORRECTION_TIMESTEP,
                leftFieldVelocity * TuningConstants.SDSDRIVETRAIN_POSE_ESTIMATION_INVERSE_TWIST_CORRECTION_TIMESTEP,
                omegaDegrees * TuningConstants.SDSDRIVETRAIN_POSE_ESTIMATION_INVERSE_TWIST_CORRECTION_TIMESTEP);

            PoseHelpers.poseExponential(this.odometryTwistCorrection);

            forwardFieldVelocity = this.odometryTwistCorrection.getFirst() / TuningConstants.SDSDRIVETRAIN_POSE_ESTIMATION_INVERSE_TWIST_CORRECTION_TIMESTEP;
            leftFieldVelocity = this.odometryTwistCorrection.getSecond() / TuningConstants.SDSDRIVETRAIN_POSE_ESTIMATION_INVERSE_TWIST_CORRECTION_TIMESTEP;
            omegaDegrees = this.odometryTwistCorrection.getThird() / TuningConstants.SDSDRIVETRAIN_POSE_ESTIMATION_INVERSE_TWIST_CORRECTION_TIMESTEP;
        }

        this.forwardFieldVelocity = forwardFieldVelocity;
        this.leftFieldVelocity = leftFieldVelocity;
        this.angularVelocity = omegaDegrees;

        this.angle += omegaDegrees * this.deltaT;
        this.xPosition += forwardFieldVelocity * this.deltaT;
        this.yPosition += leftFieldVelocity * this.deltaT;
    }

    /**
     * Basic structure to hold an angle/drive pair
     */
    private class Setpoint
    {
        public Double angle;
        public Double driveVelocity;
        public Double drivePosition;

        public Setpoint()
        {
        }
    }

    public double getPositionX()
    {
        return this.xPosition;
    }

    public double getPositionY()
    {
        return this.yPosition;
    }

    public double getForwardFieldVelocity()
    {
        return this.forwardFieldVelocity;
    }

    public double getLeftFieldVelocity()
    {
        return this.leftFieldVelocity;
    }

    public double getAngularVelocity()
    {
        return this.angularVelocity;
    }
}