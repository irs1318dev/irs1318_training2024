package frc.robot;

import frc.lib.robotprovider.LoggingType;

/**
 * Keys describing logging 
 */
public enum LoggingKey
{
    RobotState("r/state", LoggingType.String, false, 1, true),
    RobotTime("r/time", LoggingType.Number, false, 1, true),
    RobotMatch("r/match", LoggingType.String, false, 50),
    RobotCrash("r/crash", LoggingType.String, false, true),
    DriverMode("driver/mode", LoggingType.String, false, 1, true),
    DriverActiveMacros("driver/activeMacros", LoggingType.String, false, 1, true),
    DriverActiveShifts("driver/activeShifts", LoggingType.String, false),
    AutonomousSelection("auto/selected", LoggingType.String, false),
    AutonomousDSMessage("auto/dsMessage", LoggingType.String, false),
    OffboardVisionAprilTagXOffset("vision/atXOffset", LoggingType.NullableNumber, true, 1),
    OffboardVisionAprilTagYOffset("vision/atYOffset", LoggingType.NullableNumber, true, 1),
    OffboardVisionAprilTagZOffset("vision/atZOffset", LoggingType.NullableNumber, true, 1),
    OffboardVisionAprilTagYaw("vision/atYaw", LoggingType.NullableNumber, true, 1),
    OffboardVisionAprilTagPitch("vision/atPitch", LoggingType.NullableNumber, true, 1),
    OffboardVisionAprilTagRoll("vision/atRoll", LoggingType.NullableNumber, true, 1),
    OffboardVisionAprilTagId("vision/atId", LoggingType.NullableInteger, true, 1),
    OffboardVisionProcessingMode("vision/processingMode", LoggingType.Integer, false, 1),
    OffboardVisionEnableStream("vision/enableStream", LoggingType.Boolean, false, 1),
    OffboardVisionDesiredTarget("vision/desiredTarget", LoggingType.String, false, 1),
    OffboardVisionMissedHeartbeats("vision/missedHeartbeats", LoggingType.Number, true, 1),
    OffboardVisionExcessiveMissedHeartbeats("vision/missedTooManyHeartbeats", LoggingType.Boolean, false, 1),
    PowerCurrent("power/curr", LoggingType.Number, true),
    PowerCurrentFloatingAverage("power/currFltAvg", LoggingType.Number, false),
    PowerBatteryVoltage("power/battV", LoggingType.Number, true),
    // PowerBatteryVoltageFiltered("power/battVFilt", LoggingType.Number, false),
    NavxStartingAngle("navx/startingAngle", LoggingType.Number, false),
    PigeonState("pigeon/state", LoggingType.String, true),
    PigeonYaw("pigeon/yaw", LoggingType.Number, true),
    PigeonPitch("pigeon/pitch", LoggingType.Number, true),
    PigeonPitchOffset("pigeon/pitchOffset", LoggingType.Number, false),
    PigeonRollOffset("pigeon/rollOffset", LoggingType.Number, false),
    PigeonYawOffset("pigeon/yawOffset", LoggingType.Number, false),
    PigeonRoll("pigeon/roll", LoggingType.Number, true),
    PigeonStartingYaw("pigeon/startingYaw", LoggingType.Number, false),
    PigeonYawRate("pigeon/yawRate", LoggingType.Number, true),
    PigeonPitchRate("pigeon/pitchRate", LoggingType.Number, true),
    PigeonRollRate("pigeon/rollRate", LoggingType.Number, true),
    NavxConnected("navx/isConnected", LoggingType.Boolean, true),
    NavxAngle("navx/angle", LoggingType.Number, true),
    NavxPitch("navx/pitch", LoggingType.Number, true),
    NavxRoll("navx/roll", LoggingType.Number, true),
    NavxYaw("navx/yaw", LoggingType.Number, true),
    NavxX("navx/x", LoggingType.Number, true),
    NavxY("navx/y", LoggingType.Number, true),
    NavxZ("navx/z", LoggingType.Number, true),

    DriveTrainDesiredAngle("dt/angle_goal", LoggingType.Number, false),
    DriveTrainAngle("dt/angle", LoggingType.Number, false),
    DriveTrainXPosition("dt/xpos", LoggingType.Number, false, 1, true),
    DriveTrainYPosition("dt/ypos", LoggingType.Number, false, 1, true),
    DriveTrainXPositionGoal("dt/xpos_goal", LoggingType.Number, false, true),
    DriveTrainYPositionGoal("dt/ypos_goal", LoggingType.Number, false, true),
    DriveTrainAngleGoal("dt/angle_pathgoal", LoggingType.Number, false),
    DriveTrainXVelocityGoal("dt/xvel_goal", LoggingType.Number, false),
    DriveTrainYVelocityGoal("dt/yvel_goal", LoggingType.Number, false),
    DriveTrainAngleVelocityGoal("dt/anglevel_goal", LoggingType.Number, false),
    DriveTrainFieldOriented("dt/field_oriented", LoggingType.Boolean, false),
    DriveTrainMaintainOrientation("dt/maintain_orientation", LoggingType.Boolean, false),

    DriveTrainAbsoluteEncoderAngle1("dt/absenc_ang1", LoggingType.Number, true),
    DriveTrainDriveVelocity1("dt/drive_vel1", LoggingType.Number, true),
    DriveTrainDrivePosition1("dt/drive_pos1", LoggingType.Number, true),
    DriveTrainDriveError1("dt/drive_err1", LoggingType.Number, true), // SDS-only
    DriveTrainDriveVelocityGoal1("dt/drive_goal1", LoggingType.Number, false),
    DriveTrainSteerVelocity1("dt/steer_vel1", LoggingType.Number, true),
    DriveTrainSteerPosition1("dt/steer_pos1", LoggingType.Number, true), // SDS-only
    DriveTrainSteerAngle1("dt/steer_ang1", LoggingType.Number, false),
    DriveTrainSteerError1("dt/steer_err1", LoggingType.Number, true), // SDS-only
    DriveTrainSteerPositionGoal1("dt/steer_goal1", LoggingType.NullableNumber, false),
    DriveTrainSteerPositionGoal1b("dt/steer_goal1b", LoggingType.NullableNumber, false),

    DriveTrainAbsoluteEncoderAngle2("dt/absenc_ang2", LoggingType.Number, true),
    DriveTrainDriveVelocity2("dt/drive_vel2", LoggingType.Number, true),
    DriveTrainDrivePosition2("dt/drive_pos2", LoggingType.Number, true),
    DriveTrainDriveError2("dt/drive_err2", LoggingType.Number, true), // SDS-only
    DriveTrainDriveVelocityGoal2("dt/drive_goal2", LoggingType.Number, false),
    DriveTrainSteerVelocity2("dt/steer_vel2", LoggingType.Number, true),
    DriveTrainSteerPosition2("dt/steer_pos2", LoggingType.Number, true), // SDS-only
    DriveTrainSteerAngle2("dt/steer_ang2", LoggingType.Number, false),
    DriveTrainSteerError2("dt/steer_err2", LoggingType.Number, true), // SDS-only
    DriveTrainSteerPositionGoal2("dt/steer_goal2", LoggingType.NullableNumber, false),
    DriveTrainSteerPositionGoal2b("dt/steer_goal2b", LoggingType.NullableNumber, false),

    DriveTrainAbsoluteEncoderAngle3("dt/absenc_ang3", LoggingType.Number, true),
    DriveTrainDriveVelocity3("dt/drive_vel3", LoggingType.Number, true),
    DriveTrainDrivePosition3("dt/drive_pos3", LoggingType.Number, true),
    DriveTrainDriveError3("dt/drive_err3", LoggingType.Number, true), // SDS-only
    DriveTrainDriveVelocityGoal3("dt/drive_goal3", LoggingType.Number, false),
    DriveTrainSteerVelocity3("dt/steer_vel3", LoggingType.Number, true),
    DriveTrainSteerPosition3("dt/steer_pos3", LoggingType.Number, true), // SDS-only
    DriveTrainSteerAngle3("dt/steer_ang3", LoggingType.Number, false),
    DriveTrainSteerError3("dt/steer_err3", LoggingType.Number, true), // SDS-only
    DriveTrainSteerPositionGoal3("dt/steer_goal3", LoggingType.NullableNumber, false),
    DriveTrainSteerPositionGoal3b("dt/steer_goal3b", LoggingType.NullableNumber, false),

    DriveTrainAbsoluteEncoderAngle4("dt/absenc_ang4", LoggingType.Number, true),
    DriveTrainDriveVelocity4("dt/drive_vel4", LoggingType.Number, true),
    DriveTrainDrivePosition4("dt/drive_pos4", LoggingType.Number, true),
    DriveTrainDriveError4("dt/drive_err4", LoggingType.Number, true), // SDS-only
    DriveTrainDriveVelocityGoal4("dt/drive_goal4", LoggingType.Number, false),
    DriveTrainSteerVelocity4("dt/steer_vel4", LoggingType.Number, true),
    DriveTrainSteerPosition4("dt/steer_pos4", LoggingType.Number, true), // SDS-only
    DriveTrainSteerAngle4("dt/steer_ang4", LoggingType.Number, false),
    DriveTrainSteerError4("dt/steer_err4", LoggingType.Number, true), // SDS-only
    DriveTrainSteerPositionGoal4("dt/steer_goal4", LoggingType.NullableNumber, false),
    DriveTrainSteerPositionGoal4b("dt/steer_goal4b", LoggingType.NullableNumber, false),

    WristMotorVelocity("w/wrist_vel", LoggingType.Number, true),
    WristMotorAngle("w/wrist_ang", LoggingType.Number, true),
    IntakeMotorVelocity("w/intake_vel", LoggingType.Number, true),
    IntakeMotorVelocitySetpoint("w/intake_des_vel", LoggingType.Number, false),
    IntakeMotorPercentOutput("w/intake_per_out", LoggingType.Number, false),
    IntakeMotorStalled("w/intake_stalled", LoggingType.Boolean, false),
    IntakeVelocityAverage("w/intake_avg_vel", LoggingType.Number, false),
    IntakePower("w/intake_avg_power", LoggingType.Number, false),
    WristMotorSetPower("w/wrist_set_pow", LoggingType.Number, false),
    WristMotorSetPosition("w/wrist_set_pos", LoggingType.Number, false),
    WristMotorStalled("w/wrist_stalled", LoggingType.Boolean, false),
    WristVelocityAverage("w/wrist_avg_vel", LoggingType.Number, false),
    WristPower("w/wrist_avg_power", LoggingType.Number, false),

    CompressorPreassure("com/pres", LoggingType.Number, true);

    public final String value;
    public final LoggingType type;
    public final boolean isInput;
    public final int loggingFrequency;
    public final boolean shouldLogToCsv;
    private LoggingKey(String value, LoggingType type)
    {
        this(value, type, false, TuningConstants.DEFAULT_LOGGING_FREQUENCY, false);
    }

    private LoggingKey(String value, LoggingType type, boolean isInput)
    {
        this(value, type, isInput, TuningConstants.DEFAULT_LOGGING_FREQUENCY, false);
    }

    private LoggingKey(String value, LoggingType type, boolean isInput, int loggingFrequency)
    {
        this(value, type, isInput, loggingFrequency, false);
    }

    private LoggingKey(String value, LoggingType type, boolean isInput, boolean shouldLogToCsv)
    {
        this(value, type, isInput, TuningConstants.DEFAULT_LOGGING_FREQUENCY, shouldLogToCsv);
    }

    private LoggingKey(String value, LoggingType type, boolean isInput, int loggingFrequency, boolean shouldLogToCsv)
    {
        if (loggingFrequency <= 0)
        {
            loggingFrequency = TuningConstants.DEFAULT_LOGGING_FREQUENCY;
        }

        this.value = value;
        this.type = type;
        this.isInput = isInput;
        this.loggingFrequency = loggingFrequency;
        this.shouldLogToCsv = shouldLogToCsv;
    }
}
