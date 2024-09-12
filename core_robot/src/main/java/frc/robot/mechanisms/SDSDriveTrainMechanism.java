/**
 * SDSDriveTrainMechanism
 * 
 * authors: Will, Vanshika, Arushi
 * 
 * Started idk sometime in september
 * 
 * dO yOu ReMeMbEr
 * tHe 21sT nIgHt oF sEpTeMbEr
**/

package frc.robot.mechanisms;

import frc.robot.*;
import frc.lib.controllers.PIDHandler;
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
public class SDSDriveTrainMechanism implements IDriveTrainMechanism
{
    private static final int NUM_MODULES = 4;

    private static final int defaultPidSlotId = 0;
    private static final int secondaryPidSlotId = 1;

    private static final LoggingKey[] ENCODER_ANGLE_LOGGING_KEYS = { LoggingKey.DriveTrainAbsoluteEncoderAngle1, LoggingKey.DriveTrainAbsoluteEncoderAngle2, LoggingKey.DriveTrainAbsoluteEncoderAngle3, LoggingKey.DriveTrainAbsoluteEncoderAngle4 };
    private static final LoggingKey[] DRIVE_VELOCITY_LOGGING_KEYS = { LoggingKey.DriveTrainDriveVelocity1, LoggingKey.DriveTrainDriveVelocity2, LoggingKey.DriveTrainDriveVelocity3, LoggingKey.DriveTrainDriveVelocity4 };
    private static final LoggingKey[] DRIVE_POSITION_LOGGING_KEYS = { LoggingKey.DriveTrainDrivePosition1, LoggingKey.DriveTrainDrivePosition2, LoggingKey.DriveTrainDrivePosition3, LoggingKey.DriveTrainDrivePosition4 };
    private static final LoggingKey[] DRIVE_ERROR_LOGGING_KEYS = { LoggingKey.DriveTrainDriveError1, LoggingKey.DriveTrainDriveError2, LoggingKey.DriveTrainDriveError3, LoggingKey.DriveTrainDriveError4 };
    private static final LoggingKey[] STEER_VELOCITY_LOGGING_KEYS = { LoggingKey.DriveTrainSteerVelocity1, LoggingKey.DriveTrainSteerVelocity2, LoggingKey.DriveTrainSteerVelocity3, LoggingKey.DriveTrainSteerVelocity4 };
    private static final LoggingKey[] STEER_POSITION_LOGGING_KEYS = { LoggingKey.DriveTrainSteerPosition1, LoggingKey.DriveTrainSteerPosition2, LoggingKey.DriveTrainSteerPosition3, LoggingKey.DriveTrainSteerPosition4 };
    private static final LoggingKey[] STEER_ANGLE_LOGGING_KEYS = { LoggingKey.DriveTrainSteerAngle1, LoggingKey.DriveTrainSteerAngle2, LoggingKey.DriveTrainSteerAngle3, LoggingKey.DriveTrainSteerAngle4 };
    private static final LoggingKey[] STEER_ERROR_LOGGING_KEYS = { LoggingKey.DriveTrainSteerError1, LoggingKey.DriveTrainSteerError2, LoggingKey.DriveTrainSteerError3, LoggingKey.DriveTrainSteerError4 };
    private static final LoggingKey[] DRIVE_GOAL_LOGGING_KEYS = { LoggingKey.DriveTrainDriveVelocityGoal1, LoggingKey.DriveTrainDriveVelocityGoal2, LoggingKey.DriveTrainDriveVelocityGoal3, LoggingKey.DriveTrainDriveVelocityGoal4 };
    private static final LoggingKey[] STEER_GOAL_LOGGING_KEYS = { LoggingKey.DriveTrainSteerPositionGoal1, LoggingKey.DriveTrainSteerPositionGoal2, LoggingKey.DriveTrainSteerPositionGoal3, LoggingKey.DriveTrainSteerPositionGoal4 };

    private static final AnalogOperation[] STEER_SETPOINT_OPERATIONS = new AnalogOperation[] { AnalogOperation.DriveTrainPositionSteer1, AnalogOperation.DriveTrainPositionSteer2, AnalogOperation.DriveTrainPositionSteer3, AnalogOperation.DriveTrainPositionSteer4 };
    private static final AnalogOperation[] DRIVE_SETPOINT_OPERATIONS = new AnalogOperation[] { AnalogOperation.DriveTrainPositionDrive1, AnalogOperation.DriveTrainPositionDrive2, AnalogOperation.DriveTrainPositionDrive3, AnalogOperation.DriveTrainPositionDrive4 };

    // the x offsets of the swerve modules from the default center of rotation
    private final double[] moduleOffsetX;

    // the y offsets of the swerve modules from the default center of rotation
    private final double[] moduleOffsetY;

    private final double[] drivetrainSteerMotorAbsoluteOffsets;

    private final IDriver driver;
    private final ILogger logger;
    private final ITimer timer;

    private final PigeonManager imuManager;
    private final PowerManager powerManager;

    private final ITalonFX[] steerMotors;
    private final ITalonFX[] driveMotors;
    private final ICANCoder[] absoluteEncoders;

    private final PIDHandler omegaPID;
    private final boolean[] isDirectionSwapped;
    private final PIDHandler pathOmegaPID;
    private final PIDHandler pathXOffsetPID;
    private final PIDHandler pathYOffsetPID;
    private final int[] driveSlotIds;

    private final double[] driveVelocities;
    private final double[] drivePositions;
    private final double[] driveErrors;
    private final double[] steerVelocities;
    private final double[] steerPositions;
    private final double[] steerAngles;
    private final double[] steerErrors;
    private final double[] encoderAngles;

    private final Triple<Double, Double, Double> driveTwistCorrection;
    private final Triple<Double, Double, Double> odometryTwistCorrection;
    private final Setpoint[] result;

    private final SlewRateLimiter xVelocityLimiter;
    private final SlewRateLimiter yVelocityLimiter;
    private final SlewRateLimiter angularVelocityLimiter;

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
    public SDSDriveTrainMechanism(
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

        this.steerMotors = new ITalonFX[SDSDriveTrainMechanism.NUM_MODULES];
        this.driveMotors = new ITalonFX[SDSDriveTrainMechanism.NUM_MODULES];
        this.absoluteEncoders = new ICANCoder[SDSDriveTrainMechanism.NUM_MODULES];

        this.moduleOffsetX =
            new double[]
            {
                -HardwareConstants.SDSDRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE, // module 1 (front-right)
                HardwareConstants.SDSDRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE, // module 2 (front-left)
                HardwareConstants.SDSDRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE, // module 3 (back-left)
                -HardwareConstants.SDSDRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE, // module 4 (back-right)
            };

        this.moduleOffsetY =
            new double[]
            {
                -HardwareConstants.SDSDRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE, // module 1 (front-right)
                -HardwareConstants.SDSDRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE, // module 2 (front-left)
                HardwareConstants.SDSDRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE, // module 3 (back-left)
                HardwareConstants.SDSDRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE, // module 4 (back-right)
            };

        this.drivetrainSteerMotorAbsoluteOffsets =
            new double[]
            {
                TuningConstants.SDSDRIVETRAIN_STEER_MOTOR1_ABSOLUTE_OFFSET,
                TuningConstants.SDSDRIVETRAIN_STEER_MOTOR2_ABSOLUTE_OFFSET,
                TuningConstants.SDSDRIVETRAIN_STEER_MOTOR3_ABSOLUTE_OFFSET,
                TuningConstants.SDSDRIVETRAIN_STEER_MOTOR4_ABSOLUTE_OFFSET,
            };

        int[] driveMotorCanIds =
            new int[]
            {
                ElectronicsConstants.SDSDRIVETRAIN_DRIVE_MOTOR_1_CAN_ID,
                ElectronicsConstants.SDSDRIVETRAIN_DRIVE_MOTOR_2_CAN_ID,
                ElectronicsConstants.SDSDRIVETRAIN_DRIVE_MOTOR_3_CAN_ID,
                ElectronicsConstants.SDSDRIVETRAIN_DRIVE_MOTOR_4_CAN_ID
            };

        int[] steerMotorCanIds =
            new int[]
            {
                ElectronicsConstants.SDSDRIVETRAIN_STEER_MOTOR_1_CAN_ID,
                ElectronicsConstants.SDSDRIVETRAIN_STEER_MOTOR_2_CAN_ID,
                ElectronicsConstants.SDSDRIVETRAIN_STEER_MOTOR_3_CAN_ID,
                ElectronicsConstants.SDSDRIVETRAIN_STEER_MOTOR_4_CAN_ID
            };

        int[] absoluteEncoderCanIds =
            new int[]
            {
                ElectronicsConstants.SDSDRIVETRAIN_ABSOLUTE_ENCODER_1_CAN_ID,
                ElectronicsConstants.SDSDRIVETRAIN_ABSOLUTE_ENCODER_2_CAN_ID,
                ElectronicsConstants.SDSDRIVETRAIN_ABSOLUTE_ENCODER_3_CAN_ID,
                ElectronicsConstants.SDSDRIVETRAIN_ABSOLUTE_ENCODER_4_CAN_ID
            };

        boolean[] driveMotorInvert =
            new boolean[]
            {
                HardwareConstants.SDSDRIVETRAIN_DRIVE_MOTOR1_INVERT,
                HardwareConstants.SDSDRIVETRAIN_DRIVE_MOTOR2_INVERT,
                HardwareConstants.SDSDRIVETRAIN_DRIVE_MOTOR3_INVERT,
                HardwareConstants.SDSDRIVETRAIN_DRIVE_MOTOR4_INVERT
            };

        boolean[] steerMotorInvert =
            new boolean[]
            {
                HardwareConstants.SDSDRIVETRAIN_STEER_MOTOR1_INVERT,
                HardwareConstants.SDSDRIVETRAIN_STEER_MOTOR2_INVERT,
                HardwareConstants.SDSDRIVETRAIN_STEER_MOTOR3_INVERT,
                HardwareConstants.SDSDRIVETRAIN_STEER_MOTOR4_INVERT
            };

        for (int i = 0; i < SDSDriveTrainMechanism.NUM_MODULES; i++)
        {
            this.driveMotors[i] = provider.getTalonFX(driveMotorCanIds[i], ElectronicsConstants.CANIVORE_NAME);
            this.driveMotors[i].setMotorOutputSettings(driveMotorInvert[i], MotorNeutralMode.Brake);
            this.driveMotors[i].setFeedbackUpdateRate(TuningConstants.SDSDRIVETRAIN_FEEDBACK_UPDATE_RATE_HZ);
            this.driveMotors[i].setErrorUpdateRate(TuningConstants.SDSDRIVETRAIN_ERROR_UPDATE_RATE_HZ);
            this.driveMotors[i].optimizeCanbus();
            this.driveMotors[i].setPIDF(
                TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KP,
                TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KI,
                TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KD,
                TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KF,
                SDSDriveTrainMechanism.defaultPidSlotId);
            this.driveMotors[i].setPIDF(
                TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KP,
                TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KI,
                TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KD,
                TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KF,
                SDSDriveTrainMechanism.secondaryPidSlotId);
            this.driveMotors[i].setVoltageCompensation(
                TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED,
                TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION);
            this.driveMotors[i].setCurrentLimit(
                TuningConstants.SDSDRIVETRAIN_DRIVE_SUPPLY_CURRENT_LIMITING_ENABLED,
                TuningConstants.SDSDRIVETRAIN_DRIVE_SUPPLY_CURRENT_MAX,
                TuningConstants.SDSDRIVETRAIN_DRIVE_SUPPLY_TRIGGER_CURRENT,
                TuningConstants.SDSDRIVETRAIN_DRIVE_SUPPLY_TRIGGER_DURATION,
                TuningConstants.SDSDRIVETRAIN_DRIVE_STATOR_CURRENT_LIMITING_ENABLED,
                TuningConstants.SDSDRIVETRAIN_DRIVE_STATOR_CURRENT_LIMIT);
            this.driveMotors[i].setControlMode(TalonFXControlMode.Velocity);
            this.driveMotors[i].setSelectedSlot(SDSDriveTrainMechanism.defaultPidSlotId);

            this.steerMotors[i] = provider.getTalonFX(steerMotorCanIds[i], ElectronicsConstants.CANIVORE_NAME);
            this.steerMotors[i].setMotorOutputSettings(steerMotorInvert[i], MotorNeutralMode.Brake);
            this.steerMotors[i].setPIDF(
                TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_POSITION_PID_KP,
                TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_POSITION_PID_KI,
                TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_POSITION_PID_KD,
                TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_POSITION_PID_KF,
                SDSDriveTrainMechanism.defaultPidSlotId);
            this.steerMotors[i].setMotionMagicPIDVS(
                TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_MM_PID_KP,
                TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_MM_PID_KI,
                TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_MM_PID_KD,
                TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_MM_PID_KV,
                TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_MM_PID_KS,
                TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_MM_PID_CRUISE_VELOC,
                TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_MM_PID_ACCEL,
                TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_MM_PID_JERK,
                SDSDriveTrainMechanism.secondaryPidSlotId);
            this.steerMotors[i].setVoltageCompensation(
                TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED,
                TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION);
            this.steerMotors[i].setCurrentLimit(
                TuningConstants.SDSDRIVETRAIN_STEER_SUPPLY_CURRENT_LIMITING_ENABLED,
                TuningConstants.SDSDRIVETRAIN_STEER_SUPPLY_CURRENT_MAX,
                TuningConstants.SDSDRIVETRAIN_STEER_SUPPLY_TRIGGER_CURRENT,
                TuningConstants.SDSDRIVETRAIN_STEER_SUPPLY_TRIGGER_DURATION,
                TuningConstants.SDSDRIVETRAIN_STEER_STATOR_CURRENT_LIMITING_ENABLED,
                TuningConstants.SDSDRIVETRAIN_STEER_STATOR_CURRENT_LIMIT);
            this.steerMotors[i].setFeedbackUpdateRate(TuningConstants.SDSDRIVETRAIN_FEEDBACK_UPDATE_RATE_HZ);
            this.steerMotors[i].setErrorUpdateRate(TuningConstants.SDSDRIVETRAIN_ERROR_UPDATE_RATE_HZ);
            this.steerMotors[i].optimizeCanbus();
            if (TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_USE_MOTION_MAGIC)
            {
                this.steerMotors[i].setControlMode(TalonFXControlMode.MotionMagicPosition);
                this.steerMotors[i].setSelectedSlot(SDSDriveTrainMechanism.secondaryPidSlotId);
            }
            else
            {
                this.steerMotors[i].setControlMode(TalonFXControlMode.Position);
                this.steerMotors[i].setSelectedSlot(SDSDriveTrainMechanism.defaultPidSlotId);
            }

            this.absoluteEncoders[i] = provider.getCANCoder(absoluteEncoderCanIds[i], ElectronicsConstants.CANIVORE_NAME);
        }

        // prepare arrays for sensor data to re-use on each readSensors/update loop
        this.driveVelocities = new double[SDSDriveTrainMechanism.NUM_MODULES];
        this.drivePositions = new double[SDSDriveTrainMechanism.NUM_MODULES];
        this.driveErrors = new double[SDSDriveTrainMechanism.NUM_MODULES];
        this.steerVelocities = new double[SDSDriveTrainMechanism.NUM_MODULES];
        this.steerPositions = new double[SDSDriveTrainMechanism.NUM_MODULES];
        this.steerAngles = new double[SDSDriveTrainMechanism.NUM_MODULES];
        this.steerErrors = new double[SDSDriveTrainMechanism.NUM_MODULES];
        this.encoderAngles = new double[SDSDriveTrainMechanism.NUM_MODULES];

        this.isDirectionSwapped = new boolean[SDSDriveTrainMechanism.NUM_MODULES];
        this.driveSlotIds = new int[SDSDriveTrainMechanism.NUM_MODULES];

        this.omegaPID = new PIDHandler(
            TuningConstants.SDSDRIVETRAIN_OMEGA_POSITION_PID_KP,
            TuningConstants.SDSDRIVETRAIN_OMEGA_POSITION_PID_KI,
            TuningConstants.SDSDRIVETRAIN_OMEGA_POSITION_PID_KD,
            TuningConstants.SDSDRIVETRAIN_OMEGA_POSITION_PID_KF,
            TuningConstants.SDSDRIVETRAIN_OMEGA_POSITION_PID_KS,
            TuningConstants.SDSDRIVETRAIN_OMEGA_MIN_OUTPUT,
            TuningConstants.SDSDRIVETRAIN_OMEGA_MAX_OUTPUT,
            this.timer);

        this.pathOmegaPID = new PIDHandler(
            TuningConstants.SDSDRIVETRAIN_PATH_OMEGA_POSITION_PID_KP,
            TuningConstants.SDSDRIVETRAIN_PATH_OMEGA_POSITION_PID_KI,
            TuningConstants.SDSDRIVETRAIN_PATH_OMEGA_POSITION_PID_KD,
            TuningConstants.SDSDRIVETRAIN_PATH_OMEGA_POSITION_PID_KF,
            TuningConstants.SDSDRIVETRAIN_PATH_OMEGA_POSITION_PID_KS,
            TuningConstants.SDSDRIVETRAIN_PATH_OMEGA_MIN_OUTPUT,
            TuningConstants.SDSDRIVETRAIN_PATH_OMEGA_MAX_OUTPUT,
            this.timer);

        this.pathXOffsetPID = new PIDHandler(
            TuningConstants.SDSDRIVETRAIN_PATH_X_POSITION_PID_KP,
            TuningConstants.SDSDRIVETRAIN_PATH_X_POSITION_PID_KI,
            TuningConstants.SDSDRIVETRAIN_PATH_X_POSITION_PID_KD,
            TuningConstants.SDSDRIVETRAIN_PATH_X_POSITION_PID_KF,
            TuningConstants.SDSDRIVETRAIN_PATH_X_POSITION_PID_KS,
            TuningConstants.SDSDRIVETRAIN_PATH_X_MIN_OUTPUT,
            TuningConstants.SDSDRIVETRAIN_PATH_X_MAX_OUTPUT,
            this.timer);

        this.pathYOffsetPID = new PIDHandler(
            TuningConstants.SDSDRIVETRAIN_PATH_Y_POSITION_PID_KP,
            TuningConstants.SDSDRIVETRAIN_PATH_Y_POSITION_PID_KI,
            TuningConstants.SDSDRIVETRAIN_PATH_Y_POSITION_PID_KD,
            TuningConstants.SDSDRIVETRAIN_PATH_Y_POSITION_PID_KF,
            TuningConstants.SDSDRIVETRAIN_PATH_Y_POSITION_PID_KS,
            TuningConstants.SDSDRIVETRAIN_PATH_Y_MIN_OUTPUT,
            TuningConstants.SDSDRIVETRAIN_PATH_Y_MAX_OUTPUT,
            this.timer);

        this.driveTwistCorrection = new Triple<Double, Double, Double>(0.0, 0.0, 0.0);
        this.odometryTwistCorrection = new Triple<Double, Double, Double>(0.0, 0.0, 0.0);
        this.result = new Setpoint[SDSDriveTrainMechanism.NUM_MODULES];
        for (int i = 0; i < SDSDriveTrainMechanism.NUM_MODULES; i++)
        {
            this.result[i] = new Setpoint();
        }

        if (TuningConstants.SDSDRIVETRAIN_USE_TRANSLATIONAL_RATE_LIMITING)
        {
            this.xVelocityLimiter = new SlewRateLimiter(
                this.timer,
                TuningConstants.SDSDRIVETRAIN_TRANSLATIONAL_VELOCITY_MAX_NEGATIVE_RATE,
                TuningConstants.SDSDRIVETRAIN_TRANSLATIONAL_VELOCITY_MAX_POSITIVE_RATE,
                0.0);

            this.yVelocityLimiter = new SlewRateLimiter(
                this.timer,
                TuningConstants.SDSDRIVETRAIN_TRANSLATIONAL_VELOCITY_MAX_NEGATIVE_RATE,
                TuningConstants.SDSDRIVETRAIN_TRANSLATIONAL_VELOCITY_MAX_POSITIVE_RATE,
                0.0);
        }
        else
        {
            this.xVelocityLimiter = null;
            this.yVelocityLimiter = null;
        }

        if (TuningConstants.SDSDRIVETRAIN_USE_ROTATIONAL_RATE_LIMITING)
        {
            this.angularVelocityLimiter = new SlewRateLimiter(
                this.timer,
                TuningConstants.SDSDRIVETRAIN_ROTATIONAL_VELOCITY_MAX_NEGATIVE_RATE,
                TuningConstants.SDSDRIVETRAIN_ROTATIONAL_VELOCITY_MAX_POSITIVE_RATE,
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

        this.firstRun = TuningConstants.SDSDRIVETRAIN_RESET_ON_ROBOT_START;
        this.fieldOriented = TuningConstants.SDSDRIVETRAIN_FIELD_ORIENTED_ON_ROBOT_START;
        this.maintainOrientation = TuningConstants.SDSDRIVETRAIN_MAINTAIN_ORIENTATION_ON_ROBOT_START;
    }

    @Override
    public void readSensors()
    {
        for (int i = 0; i < SDSDriveTrainMechanism.NUM_MODULES; i++)
        {
            this.driveVelocities[i] = this.driveMotors[i].getVelocity();
            this.drivePositions[i] = this.driveMotors[i].getPosition();
            this.driveErrors[i] = this.driveMotors[i].getError();
            this.steerVelocities[i] = this.steerMotors[i].getVelocity();
            this.steerPositions[i] = this.steerMotors[i].getPosition();
            this.steerAngles[i] = Helpers.updateAngleRange180(this.steerPositions[i] * HardwareConstants.SDSDRIVETRAIN_STEER_TICK_DISTANCE);
            this.steerErrors[i] = this.steerMotors[i].getError();
            this.encoderAngles[i] = this.absoluteEncoders[i].getAbsolutePosition();

            this.logger.logNumber(SDSDriveTrainMechanism.DRIVE_VELOCITY_LOGGING_KEYS[i], this.driveVelocities[i]);
            this.logger.logNumber(SDSDriveTrainMechanism.DRIVE_POSITION_LOGGING_KEYS[i], this.drivePositions[i]);
            this.logger.logNumber(SDSDriveTrainMechanism.DRIVE_ERROR_LOGGING_KEYS[i], this.driveErrors[i]);
            this.logger.logNumber(SDSDriveTrainMechanism.STEER_VELOCITY_LOGGING_KEYS[i], this.steerVelocities[i]);
            this.logger.logNumber(SDSDriveTrainMechanism.STEER_POSITION_LOGGING_KEYS[i], this.steerPositions[i]);
            this.logger.logNumber(SDSDriveTrainMechanism.STEER_ANGLE_LOGGING_KEYS[i], this.steerAngles[i]);
            this.logger.logNumber(SDSDriveTrainMechanism.STEER_ERROR_LOGGING_KEYS[i], this.steerErrors[i]);
            this.logger.logNumber(SDSDriveTrainMechanism.ENCODER_ANGLE_LOGGING_KEYS[i], this.encoderAngles[i]);
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

        if (TuningConstants.SDSDRIVETRAIN_USE_ODOMETRY)
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
            for (int i = 0; i < SDSDriveTrainMechanism.NUM_MODULES; i++)
            {
                this.driveMotors[i].setPosition(0);
                double angleDifference = 360.0 * (this.encoderAngles[i] - this.drivetrainSteerMotorAbsoluteOffsets[i]);
                double tickDifference = angleDifference * HardwareConstants.SDSDRIVETRAIN_STEER_TICKS_PER_DEGREE;
                this.steerMotors[i].setPosition(tickDifference);

                this.drivePositions[i] = 0;
                this.steerPositions[i] = (int)tickDifference;
                this.steerAngles[i] = angleDifference % 360.0;
            }

            this.firstRun = false;
        }

        this.calculateSetpoints(useFieldOriented, maintainOrientation);
        for (int i = 0; i < SDSDriveTrainMechanism.NUM_MODULES; i++)
        {
            Setpoint current = this.result[i];
            Double steerSetpoint = current.angle;
            Double driveVelocitySetpoint = current.driveVelocity;
            Double drivePositionSetpoint = current.drivePosition;

            TalonFXControlMode driveControlMode = TalonFXControlMode.Neutral;
            int driveDesiredPidSlotId = SDSDriveTrainMechanism.defaultPidSlotId;
            double driveSetpoint = 0.0;
            if (driveVelocitySetpoint != null)
            {
                driveSetpoint = driveVelocitySetpoint;
                driveControlMode = TalonFXControlMode.Velocity;
                driveDesiredPidSlotId = SDSDriveTrainMechanism.defaultPidSlotId;
            }
            else if (drivePositionSetpoint != null)
            {
                driveSetpoint = drivePositionSetpoint;
                driveControlMode = TalonFXControlMode.Position;
                driveDesiredPidSlotId = SDSDriveTrainMechanism.secondaryPidSlotId;
            }

            this.logger.logNumber(SDSDriveTrainMechanism.DRIVE_GOAL_LOGGING_KEYS[i], driveSetpoint);
            this.driveMotors[i].setControlMode(driveControlMode);
            if (driveControlMode != TalonFXControlMode.Neutral)
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

            if (steerSetpoint != null)
            {
                this.logger.logNumber(SDSDriveTrainMechanism.STEER_GOAL_LOGGING_KEYS[i], steerSetpoint);
                this.steerMotors[i].set(steerSetpoint);
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
        for (int i = 0; i < SDSDriveTrainMechanism.NUM_MODULES; i++)
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
            for (int i = 0; i < SDSDriveTrainMechanism.NUM_MODULES; i++)
            {
                this.result[i].driveVelocity = null;
                if (maintainPositionMode)
                {
                    this.result[i].drivePosition = this.driver.getAnalog(SDSDriveTrainMechanism.DRIVE_SETPOINT_OPERATIONS[i]);
                }
                else
                {
                    this.result[i].drivePosition = null;
                }

                double moduleSteerPositionGoal = this.driver.getAnalog(SDSDriveTrainMechanism.STEER_SETPOINT_OPERATIONS[i]);
                double currentAngle = this.steerPositions[i] * HardwareConstants.SDSDRIVETRAIN_STEER_TICK_DISTANCE;
                AnglePair anglePair = AnglePair.getClosestAngle(moduleSteerPositionGoal, currentAngle, true);
                moduleSteerPositionGoal = anglePair.getAngle() * TuningConstants.SDSDRIVETRAIN_STEER_MOTOR_POSITION_PID_KS;
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
                centerVelocityLeftRaw *= TuningConstants.SDSDRIVETRAIN_SLOW_MODE_MAX_VELOCITY;
                centerVelocityForwardRaw *= TuningConstants.SDSDRIVETRAIN_SLOW_MODE_MAX_VELOCITY;
            }
            else
            {
                centerVelocityLeftRaw *= TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY;
                centerVelocityForwardRaw *= TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY;
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
                    omega *= TuningConstants.SDSDRIVETRAIN_SLOW_MODE_TURN_SCALE;
                }
                else
                {
                    omega *= TuningConstants.SDSDRIVETRAIN_TURN_SCALE;
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
                        if (Math.abs(centerVelocityForward) + Math.abs(centerVelocityLeft) < TuningConstants.SDSDRIVETRAIN_STATIONARY_VELOCITY)
                        {
                            skipTurn = TuningConstants.SDSDRIVETRAIN_TURN_APPROXIMATION_STATIONARY != 0.0 && Helpers.WithinDelta(this.desiredYaw, this.robotYaw, TuningConstants.SDSDRIVETRAIN_TURN_APPROXIMATION_STATIONARY);
                        }
                        else
                        {
                            skipTurn = TuningConstants.SDSDRIVETRAIN_TURN_APPROXIMATION != 0.0 && Helpers.WithinDelta(this.desiredYaw, this.robotYaw, TuningConstants.SDSDRIVETRAIN_TURN_APPROXIMATION);
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
        for (int i = 0; i < SDSDriveTrainMechanism.NUM_MODULES; i++)
        {
            double moduleVelocityLeft = centerVelocityLeft - omega * (this.moduleOffsetY[i] + rotationCenterB);
            double moduleVelocityForward = centerVelocityForward - omega * (this.moduleOffsetX[i] + rotationCenterA);

            Double moduleSteerPositionGoal;
            double moduleDriveVelocityGoal;
            if (TuningConstants.SDSDRIVETRAIN_SKIP_ANGLE_ON_ZERO_VELOCITY
                    && Helpers.WithinDelta(moduleVelocityLeft, 0.0, TuningConstants.SDSDRIVETRAIN_SKIP_ANGLE_ON_ZERO_DELTA)
                    && Helpers.WithinDelta(moduleVelocityForward, 0.0, TuningConstants.SDSDRIVETRAIN_SKIP_ANGLE_ON_ZERO_DELTA))
            {
                moduleDriveVelocityGoal = 0.0;
                moduleSteerPositionGoal = null;
            }
            else
            {
                moduleDriveVelocityGoal = Math.sqrt(moduleVelocityLeft * moduleVelocityLeft + moduleVelocityForward * moduleVelocityForward);

                moduleSteerPositionGoal = Helpers.atan2d(moduleVelocityLeft, moduleVelocityForward);
                double currentAngle = this.steerPositions[i] * HardwareConstants.SDSDRIVETRAIN_STEER_TICK_DISTANCE;
                AnglePair anglePair = AnglePair.getClosestAngle(moduleSteerPositionGoal, currentAngle, true);
                moduleSteerPositionGoal = anglePair.getAngle() * TuningConstants.SDSDRIVETRAIN_STEER_MOTOR_POSITION_PID_KS;
                this.isDirectionSwapped[i] = anglePair.getSwapDirection();

                if (maxModuleDriveVelocityGoal < moduleDriveVelocityGoal)
                {
                    maxModuleDriveVelocityGoal = moduleDriveVelocityGoal;
                }

                moduleDriveVelocityGoal *= HardwareConstants.SDSDRIVETRAIN_DRIVE_INCHES_PER_SECOND_TO_MOTOR_VELOCITY;
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
        if (maxModuleDriveVelocityGoal > TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY)
        {
            // divide by percentage is interchangeable with multiply by inverse-percentage
            double invPercentage = TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY / maxModuleDriveVelocityGoal;
            for (int i = 0; i < SDSDriveTrainMechanism.NUM_MODULES; i++)
            {
                this.result[i].driveVelocity *= invPercentage;
            }
        }

        if (TuningConstants.SDSDRIVETRAIN_USE_OVERCURRENT_ADJUSTMENT)
        {
            CurrentLimiting value = this.powerManager.getCurrentLimitingValue();
            if (value != CurrentLimiting.Normal)
            {
                double currentLimitingMultiplier;
                if (value == CurrentLimiting.OverCurrent)
                {
                    currentLimitingMultiplier = TuningConstants.SDSDRIVETRAIN_OVERCURRENT_ADJUSTMENT;
                }
                else // if (value == CurrentLimiting.OverCurrentHigh)
                {
                    currentLimitingMultiplier = TuningConstants.SDSDRIVETRAIN_OVERCURRENT_HIGH_ADJUSTMENT;
                }

                for (int i = 0; i < SDSDriveTrainMechanism.NUM_MODULES; i++)
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
        double rightRobotVelocity1 = -Helpers.sind(this.steerAngles[0]) * HardwareConstants.SDSDRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND * this.driveVelocities[0];
        double rightRobotVelocity2 = -Helpers.sind(this.steerAngles[1]) * HardwareConstants.SDSDRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND * this.driveVelocities[1];
        double rightRobotVelocity3 = -Helpers.sind(this.steerAngles[2]) * HardwareConstants.SDSDRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND * this.driveVelocities[2];
        double rightRobotVelocity4 = -Helpers.sind(this.steerAngles[3]) * HardwareConstants.SDSDRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND * this.driveVelocities[3];

        double forwardRobotVelocity1 = Helpers.cosd(this.steerAngles[0]) * HardwareConstants.SDSDRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND * this.driveVelocities[0];
        double forwardRobotVelocity2 = Helpers.cosd(this.steerAngles[1]) * HardwareConstants.SDSDRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND * this.driveVelocities[1];
        double forwardRobotVelocity3 = Helpers.cosd(this.steerAngles[2]) * HardwareConstants.SDSDRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND * this.driveVelocities[2];
        double forwardRobotVelocity4 = Helpers.cosd(this.steerAngles[3]) * HardwareConstants.SDSDRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND * this.driveVelocities[3];

        // rightRobotVelocity = (rightRobotVelocity1 + rightRobotVelocity2 + rightRobotVelocity3 + rightRobotVelocity4) / 4.0;
        // forwardRobotVelocity = (forwardRobotVelocity1 + forwardRobotVelocity2 + forwardRobotVelocity3 + forwardRobotVelocity4) / 4.0;

        double a = -0.5 * (rightRobotVelocity3 + rightRobotVelocity4);
        double b = -0.5 * (rightRobotVelocity1 + rightRobotVelocity2);
        double c = 0.5 * (forwardRobotVelocity1 + forwardRobotVelocity4);
        double d = 0.5 * (forwardRobotVelocity2 + forwardRobotVelocity3);

        double omegaRadians1 = (b - a) * HardwareConstants.SDSDRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE_INV;
        double omegaRadians2 = (c - d) * HardwareConstants.SDSDRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE_INV;
        double omegaRadians = 0.5 * (omegaRadians1 + omegaRadians2);

        double rightRobotVelocityA = omegaRadians * HardwareConstants.SDSDRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE + a;
        double rightRobotVelocityB = -omegaRadians * HardwareConstants.SDSDRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE + b;
        leftRobotVelocity = 0.5 * (rightRobotVelocityA + rightRobotVelocityB);

        double forwardRobotVelocityA = omegaRadians * HardwareConstants.SDSDRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE + c;
        double forwardRobotVelocityB = -omegaRadians * HardwareConstants.SDSDRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE + d;
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