package frc.robot;

import java.util.List;
import java.util.stream.Collectors;

/**
 * All constants related to tuning the operation of the robot.
 * 
 */
public class TuningConstants
{
    public static final boolean COMPETITION_ROBOT = false;
    public static final boolean USE_ADVANTAGE_KIT = true;
    public static final boolean LOG_NULL_WHILE_DISABLED = true;
    public static final boolean RETREIVE_PDH_FIRST = true;

    public static boolean THROW_EXCEPTIONS = true;
    public static boolean LOG_EXCEPTIONS = true;
    public static double LOOP_DURATION = 0.02; // we expect the robot's main loop to run at roughly ~50 Hz, or 1 update per 20ms (0.02s)
    public static int LOOPS_PER_SECOND = 50; // we expect the robot's main loop to run at roughly ~50 Hz, or 1 update per 20ms (0.02s)

    public static final boolean EXPECT_UNUSED_JOYSTICKS = true;
    public static final boolean PERFORM_COSTLY_TASKS_WHILE_DISABLED = true;

    //================================================== Magic Values ==============================================================

    public static final double MAGIC_NULL_VALUE = -1318.0;
    public static final double ZERO = 0.0;
    public static final double ENDGAME_START_TIME = 30.0;
    public static final double ENDGAME_CLIMB_TIME = 5.0;

    //================================================== Logging  ==============================================================

    public static final int CALENDAR_YEAR = 2024;
    public static final boolean LOG_TO_FILE = false; // TuningConstants.COMPETITION_ROBOT;
    public static final boolean LOG_FILE_ONLY_COMPETITION_MATCHES = false;
    public static final long LOG_FILE_REQUIRED_FREE_SPACE = 50 * 1024 * 1024; // require at least 50 MB of space
    public static final int LOG_FLUSH_THRESHOLD = 25;
    public static final boolean USE_LOGGING_FREQUENCY = true; // TuningConstants.COMPETITION_ROBOT;
    public static final int DEFAULT_LOGGING_FREQUENCY = 10; // number of entries to ignore between logging

    //================================================== Autonomous ==============================================================

    public static final boolean TRAJECTORY_FORCE_BUILD = false;

    //================================================= Power ======================================================

    public static final double POWER_OVERCURRENT_TRACKING_DURATION = 5.0; // duration of time to keep track of the average current
    public static final double POWER_OVERCURRENT_TRACKING_MAX_VALUE = 1000.0; // 1000 amos is an unrealistic max value to use for overcurrent
    public static final double POWER_OVERCURRENT_SAMPLES_PER_LOOP = 1.0; // we may want to increase this if we find our update loop duration isn't very consistent...
    public static final double POWER_OVERCURRENT_SAMPLES_PER_SECOND = TuningConstants.LOOPS_PER_SECOND * TuningConstants.POWER_OVERCURRENT_SAMPLES_PER_LOOP;
    public static final double POWER_OVERCURRENT_THRESHOLD = 140.0;
    public static final double POWER_OVERCURREHT_HIGH_THRESHOLD = 180.0;

    //================================================= Macros/Vision ======================================================

    public static final double VISION_ODOMETRY_ACCURACY_TRESHOLD_RANGE = 30;
    public static final boolean SDSDRIVETRAIN_USE_VISION = true;

    //=========================================== 2024 AprilTag Location guide ==============================================
    //// | TAG                                 |  ID  |    X    |    Y    |   Z    | THETA |
    //// |-------------------------------------|------|---------|---------|--------|-------|
    //// | APRILTAG_BLUE_SOURCE_RIGHT_ID       |   1  |  268.07 |    6.26 |  53.38 | 120.0 |
    //// | APRILTAG_BLUE_SOURCE_LEFT_ID        |   2  |  311.59 |   31.37 |  53.38 | 120.0 |
    //// | APRILTAG_RED_SPEAKER_OFFCENTER_ID   |   3  |  327.12 |  192.75 |  57.13 | 180.0 |
    //// | APRILTAG_RED_SPEAKER_CENTER_ID      |   4  |  327.12 |  215.0  |  57.13 | 180.0 |
    //// | APRILTAG_RED_AMP_ID                 |   5  |  253.16 |  319.58 |  53.38 | 270.0 |
    //// | APRILTAG_BLUE_AMP_ID                |   6  | -253.16 |  319.58 |  53.38 | 270.0 |
    //// | APRILTAG_BLUE_SPEAKER_CENTER_ID     |   7  | -327.12 |  215.0  |  57.13 |   0.0 |
    //// | APRILTAG_BLUE_SPEAKER_OFFCENTER_ID  |   8  | -327.12 |  192.75 |  57.13 |   0.0 |
    //// | ARPILTAG_RED_SOURCE_RIGHT_ID        |   9  | -311.59 |   31.37 |  53.38 |  60.0 |
    //// | APRILTAG_RED_SOURCE_LEFT_ID         |  10  | -268.07 |    6.26 |  53.38 |  60.0 |
    //// | APRILTAG_RED_STAGE_LEFT_ID          |  11  |  143.0  |  142.77 |  52.0  | 300.0 |
    //// | APRILTAG_RED_STAGE_RIGHT_ID         |  12  |  143.0  |  173.68 |  52.0  |  60.0 |
    //// | APRILTAG_RED_CENTER_STAGE_ID        |  13  |  116.13 |  158.5  |  52.0  | 180.0 |
    //// | APRILTAG_BLUE_CENTER_STAGE_ID       |  14  | -116.13 |  158.5  |  52.0  |   0.0 |
    //// | APRILTAG_BLUE_STAGE_LEFT_ID         |  15  | -143.0  |  173.68 |  52.0  | 120.0 |
    //// | APRILTAG_BLUE_STAGE_RIGHT_ID        |  16  | -143.0  |  142.77 |  52.0  | 240.0 |
    //// 
    //// Conversion from FIRST's published values: (x - 325.615, y - ~3.42, z, rot)
    public static final double APRILTAG_RED_SPEAKER_X_POSITION = 327.12;
    public static final double APRILTAG_RED_SPEAKER_Y_POSITION = 215.0;
    public static final double APRILTAG_BLUE_SPEAKER_X_POSITION = -327.12;
    public static final double APRILTAG_BLUE_SPEAKER_Y_POSITION = 215.0;

    public static final int APRILTAG_BLUE_SOURCE_RIGHT_ID = 1;
    public static final int APRILTAG_BLUE_SOURCE_LEFT_ID = 2;
    public static final int APRILTAG_RED_SPEAKER_OFFCENTER_ID = 3;
    public static final int APRILTAG_RED_SPEAKER_CENTER_ID = 4;
    public static final int APRILTAG_RED_AMP_ID = 5;
    public static final int APRILTAG_BLUE_AMP_ID = 6;
    public static final int APRILTAG_BLUE_SPEAKER_CENTER_ID = 7;
    public static final int APRILTAG_BLUE_SPEAKER_OFFCENTER_ID = 8;
    public static final int APRILTAG_RED_SOURCE_RIGHT_ID = 9;
    public static final int APRILTAG_RED_SOURCE_LEFT_ID = 10;
    public static final int APRILTAG_RED_STAGE_LEFT_ID = 11;
    public static final int APRILTAG_RED_STAGE_RIGHT_ID = 12;
    public static final int APRILTAG_RED_CENTER_STAGE_ID = 13;
    public static final int APRILTAG_BLUE_CENTER_STAGE_ID = 14;
    public static final int APRILTAG_BLUE_STAGE_LEFT_ID = 15;
    public static final int APRILTAG_BLUE_STAGE_RIGHT_ID = 16;

    public static final List<Integer> VISION_SPEAKER_BLUE_APRILTAGS = List.of(TuningConstants.APRILTAG_BLUE_SPEAKER_CENTER_ID, TuningConstants.APRILTAG_BLUE_SPEAKER_OFFCENTER_ID);
    public static final String VISION_SPEAKER_BLUE_STRING = TuningConstants.VISION_SPEAKER_BLUE_APRILTAGS.stream().map((i) -> Integer.toString(i)).collect(Collectors.joining(","));
    public static final List<Integer> VISION_SPEAKER_RED_APRILTAGS = List.of(TuningConstants.APRILTAG_RED_SPEAKER_CENTER_ID, TuningConstants.APRILTAG_RED_SPEAKER_OFFCENTER_ID);
    public static final String VISION_SPEAKER_RED_STRING = TuningConstants.VISION_SPEAKER_RED_APRILTAGS.stream().map((i) -> Integer.toString(i)).collect(Collectors.joining(","));

    public static final List<Integer> VISION_STAGE_BLUE_APRILTAGS = List.of(TuningConstants.APRILTAG_BLUE_STAGE_LEFT_ID, TuningConstants.APRILTAG_BLUE_CENTER_STAGE_ID, TuningConstants.APRILTAG_BLUE_STAGE_RIGHT_ID);
    public static final String VISION_STAGE_BLUE_STRING = TuningConstants.VISION_STAGE_BLUE_APRILTAGS.stream().map((i) -> Integer.toString(i)).collect(Collectors.joining(","));
    public static final List<Integer> VISION_STAGE_RED_APRILTAGS = List.of(TuningConstants.APRILTAG_RED_STAGE_LEFT_ID, TuningConstants.APRILTAG_RED_CENTER_STAGE_ID, TuningConstants.APRILTAG_RED_STAGE_RIGHT_ID);
    public static final String VISION_STAGE_RED_STRING = TuningConstants.VISION_STAGE_RED_APRILTAGS.stream().map((i) -> Integer.toString(i)).collect(Collectors.joining(","));

    public static final List<Integer> VISION_AMP_BLUE_APRILTAGS = List.of(TuningConstants.APRILTAG_BLUE_AMP_ID);
    public static final String VISION_AMP_BLUE_STRING = TuningConstants.VISION_AMP_BLUE_APRILTAGS.stream().map((i) -> Integer.toString(i)).collect(Collectors.joining(","));
    public static final List<Integer> VISION_AMP_RED_APRILTAGS = List.of(TuningConstants.APRILTAG_RED_AMP_ID);
    public static final String VISION_AMP_RED_STRING = TuningConstants.VISION_AMP_RED_APRILTAGS.stream().map((i) -> Integer.toString(i)).collect(Collectors.joining(","));

    // Finding AprilTags to determine if theres enough valid data to translate 
    public static final int TAGS_MISSED_THRESHOLD = 30;
    public static final int TAGS_FOUND_THRESHOLD = 5;
    public static final double ACCEPTABLE_RANGE_IN_X_AND_Y_FOR_ALIGNMENT_TRANSLATE = 1.0; // in inches

    // Acceptable vision centering range values in degrees
    public static final double MAX_PID_TURNING_RANGE_DEGREES = 2.0;

    // How long the robot system must remain centered on the target when using time
    public static final double PID_TURNING_DURATION = 0.75;

    // Acceptable vision distance from tape in inches (as measured by vision system)
    public static final double MAX_VISION_ACCEPTABLE_FORWARD_DISTANCE = 1.75;
    public static final double MAX_VISION_ACCEPTABLE_STRAFE_DISTANCE = 0.7;

    // Acceptable vision distance from tape in angles 
    public static final double MAX_VISION_ACCEPTABLE_MOVING_RR_ANGLE_ERROR = 4.0;

    // PID settings for Centering the robot on a vision target from one stationary place, based on a single sample
    public static final double STATIONARY_SINGLE_TURNING_PID_KP = 0.027;
    public static final double STATIONARY_SINGLE_TURNING_PID_KI = 0.0;
    public static final double STATIONARY_SINGLE_TURNING_PID_KD = 0.01;
    public static final double STATIONARY_SINGLE_TURNING_PID_KF = 0.0;
    public static final double STATIONARY_SINGLE_TURNING_PID_KS = 1.0;
    public static final double STATIONARY_SINGLE_TURNING_PID_MIN = -0.4;
    public static final double STATIONARY_SINGLE_TURNING_PID_MAX = 0.4;

    // PID settings for Centering the robot on a vision target from one stationary place, based on continuous samples
    public static final double STATIONARY_CONTINUOUS_TURNING_PID_KP = 0.027;
    public static final double STATIONARY_CONTINUOUS_TURNING_PID_KI = 0.0;
    public static final double STATIONARY_CONTINUOUS_TURNING_PID_KD = 0.01;
    public static final double STATIONARY_CONTINUOUS_TURNING_PID_KF = 0.0;
    public static final double STATIONARY_CONTINUOUS_TURNING_PID_KS = 1.0;
    public static final double STATIONARY_CONTINUOUS_TURNING_PID_MIN = -0.4;
    public static final double STATIONARY_CONTINUOUS_TURNING_PID_MAX = 0.4;

    // PID settings for rotating the robot based on a vision target while in-motion
    public static final double VISION_MOVING_TURNING_PID_KP = 0.012;
    public static final double VISION_MOVING_TURNING_PID_KI = 0.0;
    public static final double VISION_MOVING_TURNING_PID_KD = 0.0;
    public static final double VISION_MOVING_TURNING_PID_KF = 0.0;
    public static final double VISION_MOVING_TURNING_PID_KS = 1.0;
    public static final double VISION_MOVING_TURNING_PID_MIN = -0.3;
    public static final double VISION_MOVING_TURNING_PID_MAX = 0.3;

    // PID settings for translating the robot based on a vision target
    public static final double VISION_MOVING_PID_KP = 0.015;
    public static final double VISION_MOVING_PID_KI = 0.0;
    public static final double VISION_MOVING_PID_KD = 0.0;
    public static final double VISION_MOVING_PID_KF = 0.0;
    public static final double VISION_MOVING_PID_KS = 1.0;
    public static final double VISION_MOVING_PID_MIN = -0.3;
    public static final double VISION_MOVING_PID_MAX = 0.3;

    // PID settings for translating the robot based on a vision target
    public static final double VISION_AT_TRANSLATION_X_PID_KP = 0.023;
    public static final double VISION_AT_TRANSLATION_X_PID_KI = 0.0;
    public static final double VISION_AT_TRANSLATION_X_PID_KD = 0.0;
    public static final double VISION_AT_TRANSLATION_X_PID_KF = 0.0;
    public static final double VISION_AT_TRANSLATION_X_PID_KS = 1.0;
    public static final double VISION_AT_TRANSLATION_X_PID_MIN = -0.3;
    public static final double VISION_AT_TRANSLATION_X_PID_MAX = 0.3;

    // PID settings for translating the robot based on a vision target
    public static final double VISION_AT_TRANSLATION_Y_PID_KP = 0.023;
    public static final double VISION_AT_TRANSLATION_Y_PID_KI = 0.0;
    public static final double VISION_AT_TRANSLATION_Y_PID_KD = 0.0;
    public static final double VISION_AT_TRANSLATION_Y_PID_KF = 0.0;
    public static final double VISION_AT_TRANSLATION_Y_PID_KS = 1.0;
    public static final double VISION_AT_TRANSLATION_Y_PID_MIN = -0.3;
    public static final double VISION_AT_TRANSLATION_Y_PID_MAX = 0.3;

    // PID settings for translating the robot slowly based on a vision target
    public static final double VISION_SLOW_MOVING_PID_KP = 0.013;
    public static final double VISION_SLOW_MOVING_PID_KI = 0.0;
    public static final double VISION_SLOW_MOVING_PID_KD = 0.0;
    public static final double VISION_SLOW_MOVING_PID_KF = 0.0;
    public static final double VISION_SLOW_MOVING_PID_KS = 1.0;
    public static final double VISION_SLOW_MOVING_PID_MIN = -0.3;
    public static final double VISION_SLOW_MOVING_PID_MAX = 0.3;

    // PID settings for translating the robot quickly based on a vision target
    public static final double VISION_FAST_MOVING_PID_KP = 0.17;
    public static final double VISION_FAST_MOVING_PID_KI = 0.0;
    public static final double VISION_FAST_MOVING_PID_KD = 0.0;
    public static final double VISION_FAST_MOVING_PID_KF = 0.0;
    public static final double VISION_FAST_MOVING_PID_KS = 1.0;
    public static final double VISION_FAST_MOVING_PID_MIN = -0.45;
    public static final double VISION_FAST_MOVING_PID_MAX = 0.45;

    public static final int VISION_MISSED_HEARTBEAT_THRESHOLD = 50;

    public static final double ORIENTATION_TURN_THRESHOLD = 2.0; // number of degrees off at which point we give up trying to face an angle

    //================================================== Driver Feedback ========================================================

    public static final double ENDGAME_RUMBLE = 20.0;

    public static final int LED_START = 0;
    public static final int LED_COUNT = 8;
    public static final int LED_STRIP_LED_START = TuningConstants.LED_COUNT;
    public static final int LED_STRIP_LED_COUNT = 60; // 60 LEDs per meter-long strip from CTRE
    public static final int TOTAL_NUMBER_LEDS = TuningConstants.LED_COUNT + TuningConstants.LED_STRIP_LED_COUNT;

    // IRS1318 Purple color
    public static final int INDICATOR_PURPLE_COLOR_RED = 101;
    public static final int INDICATOR_PURPLE_COLOR_GREEN = 34;
    public static final int INDICATOR_PURPLE_COLOR_BLUE = 129;
    public static final int INDICATOR_PURPLE_COLOR_WHITE = 0;

    // Bright Yellow color
    public static final int INDICATOR_YELLOW_COLOR_RED = 255;
    public static final int INDICATOR_YELLOW_COLOR_GREEN = 255;
    public static final int INDICATOR_YELLOW_COLOR_BLUE = 0;
    public static final int INDICATOR_YELLOW_COLOR_WHITE = 0;

    // Bright Green color
    public static final int INDICATOR_GREEN_COLOR_RED = 0;
    public static final int INDICATOR_GREEN_COLOR_GREEN = 255;
    public static final int INDICATOR_GREEN_COLOR_BLUE = 0;
    public static final int INDICATOR_GREEN_COLOR_WHITE = 0;

    // Bright Red color
    public static final int INDICATOR_RED_COLOR_RED = 255;
    public static final int INDICATOR_RED_COLOR_GREEN = 0;
    public static final int INDICATOR_RED_COLOR_BLUE = 0;
    public static final int INDICATOR_RED_COLOR_WHITE = 0;

    // Blue
    public static final int INDICATOR_BLUE_COLOR_RED = 0;
    public static final int INDICATOR_BLUE_COLOR_GREEN = 0;
    public static final int INDICATOR_BLUE_COLOR_BLUE = 255;
    public static final int INDICATOR_BLUE_COLOR_WHITE = 0;

    // Orange
    public static final int INDICATOR_ORANGE_COLOR_RED = 255;
    public static final int INDICATOR_ORANGE_COLOR_GREEN = 165;
    public static final int INDICATOR_ORANGE_COLOR_BLUE = 0;
    public static final int INDICATOR_ORANGE_COLOR_WHITE = 0;

    // Rainbow
    public static final int INDICATOR_RAINBOW_BRIGHTNESS = 1;
    public static final double INDICATOR_RAINBOW_SPEED = 0.25;
    public static final boolean INDICATOR_RAINBOW_REVERSE_DIRECTION = false;

    // No color
    public static final int INDICATOR_OFF_COLOR_RED = 0;
    public static final int INDICATOR_OFF_COLOR_GREEN = 0;
    public static final int INDICATOR_OFF_COLOR_BLUE = 0;
    public static final int INDICATOR_OFF_COLOR_WHITE = 0;

    //================================================== SDS DriveTrain ==============================================================

    public static final boolean SDSDRIVETRAIN_STEER_MOTORS_USE_MOTION_MAGIC = true;
    public static final boolean SDSDRIVETRAIN_USE_POSE_ESTIMATION_INVERSE_TWIST_CORRECTION = true;
    public static final double SDSDRIVETRAIN_POSE_ESTIMATION_INVERSE_TWIST_CORRECTION_TIMESTEP = 0.02;

    public static final boolean SDSDRIVETRAIN_USE_ODOMETRY = true;
    public static final boolean SDSDRIVETRAIN_RESET_ON_ROBOT_START = true;
    public static final boolean SDSDRIVETRAIN_FIELD_ORIENTED_ON_ROBOT_START = true;
    public static final boolean SDSDRIVETRAIN_MAINTAIN_ORIENTATION_ON_ROBOT_START = true;

    public static final double SDSDRIVETRAIN_STEER_MOTOR1_ABSOLUTE_OFFSET = TuningConstants.COMPETITION_ROBOT ? 0.3046875 : 0.17212; // -0.198486 + 0.5; // rotations
    public static final double SDSDRIVETRAIN_STEER_MOTOR2_ABSOLUTE_OFFSET = TuningConstants.COMPETITION_ROBOT ? 0.3129882 : 0.15698;//-0.186768 + 0.5; // rotations
    public static final double SDSDRIVETRAIN_STEER_MOTOR3_ABSOLUTE_OFFSET = TuningConstants.COMPETITION_ROBOT ? 0.3864746 : -0.28979;//0.46337890625; // rotations
    public static final double SDSDRIVETRAIN_STEER_MOTOR4_ABSOLUTE_OFFSET = TuningConstants.COMPETITION_ROBOT ? -0.1572265 : -0.35327;//0.336670 + 0.5; // rotations

    public static final boolean SDSDRIVETRAIN_USE_TRANSLATIONAL_RATE_LIMITING = true;
    public static final double SDSDRIVETRAIN_TRANSLATIONAL_VELOCITY_MAX_NEGATIVE_RATE = -4.0 * TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY;
    public static final double SDSDRIVETRAIN_TRANSLATIONAL_VELOCITY_MAX_POSITIVE_RATE = 4.0 * TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY;
    public static final boolean SDSDRIVETRAIN_USE_ROTATIONAL_RATE_LIMITING = true;
    public static final double SDSDRIVETRAIN_ROTATIONAL_VELOCITY_MAX_NEGATIVE_RATE = -4.0 * TuningConstants.SDSDRIVETRAIN_TURN_SCALE;
    public static final double SDSDRIVETRAIN_ROTATIONAL_VELOCITY_MAX_POSITIVE_RATE = 4.0 * TuningConstants.SDSDRIVETRAIN_TURN_SCALE;

    // Position PID (angle) per-module
    public static final double SDSDRIVETRAIN_STEER_MOTOR_POSITION_PID_KS = HardwareConstants.SDSDRIVETRAIN_STEER_TICKS_PER_DEGREE;

    public static final double SDSDRIVETRAIN_STEER_MOTORS_POSITION_PID_KP = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_COMP_POSITION_PID_KP : TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_POSITION_PID_KP;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_POSITION_PID_KI = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_COMP_POSITION_PID_KI : TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_POSITION_PID_KI;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_POSITION_PID_KD = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_COMP_POSITION_PID_KD : TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_POSITION_PID_KD;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_POSITION_PID_KF = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_COMP_POSITION_PID_KF : TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_POSITION_PID_KF;

    public static final double SDSDRIVETRAIN_STEER_MOTORS_MM_PID_KP = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_KP : TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_KP;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_MM_PID_KI = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_KI : TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_KI;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_MM_PID_KD = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_KD : TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_KD;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_MM_PID_KV = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_KV : TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_KV;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_MM_PID_KS = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_KS : TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_KS;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_MM_PID_CRUISE_VELOC = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_CRUISE_VELOC : TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_CRUISE_VELOC;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_MM_PID_ACCEL = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_ACCEL : TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_ACCEL;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_MM_PID_JERK = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_JERK : TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_JERK;

    // STEER PRACTICE
    public static final double SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_POSITION_PID_KP = 1.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_POSITION_PID_KI = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_POSITION_PID_KD = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_POSITION_PID_KF = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);

    // STEER RPM ~107.0 was highest speed at full throttle FF on blocks
    public static final double SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_KP = 0.3333333 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_KI = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_KD = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_KV = 0.00934579 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_KS = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_CRUISE_VELOC = 100.0;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_ACCEL = 600.0;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_JERK = 9999.0;

    // STEER COMP
    public static final double SDSDRIVETRAIN_STEER_MOTORS_COMP_POSITION_PID_KP = 1.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_COMP_POSITION_PID_KI = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_COMP_POSITION_PID_KD = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_COMP_POSITION_PID_KF = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);

    // STEER RPM ~107.0 was highest speed at full throttle FF on blocks
    public static final double SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_KP = 0.3333333 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_KI = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_KD = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_KV = 0.00934579 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_KS = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_CRUISE_VELOC = 100.0;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_ACCEL = 600.0;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_JERK = 9999.0;

    // Velocity PID (drive) per-module
    public static final double SDSDRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KS = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_DRIVE_MOTOR_COMP_VELOCITY_PID_KS : TuningConstants.SDSDRIVETRAIN_DRIVE_MOTOR_PRACTICE_VELOCITY_PID_KS; // RPM ~110.0 was highest speed at full throttle FF on blocks

    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KP = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_COMP_VELOCITY_PID_KP : TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_VELOCITY_PID_KP;
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KI = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_COMP_VELOCITY_PID_KI : TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_VELOCITY_PID_KI;
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KD = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_COMP_VELOCITY_PID_KD : TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_VELOCITY_PID_KD;
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KF = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_COMP_VELOCITY_PID_KF : TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_VELOCITY_PID_KF;

    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KP = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_COMP_POSITION_PID_KP : TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_POSITION_PID_KP;
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KI = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_COMP_POSITION_PID_KI : TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_POSITION_PID_KI;
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KD = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_COMP_POSITION_PID_KD : TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_POSITION_PID_KD;
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KF = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_COMP_POSITION_PID_KF : TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_POSITION_PID_KF;

    // DRIVE PRACTICE
    public static final double SDSDRIVETRAIN_DRIVE_MOTOR_PRACTICE_VELOCITY_PID_KS = 88.0; // RPM ~110.0 was highest speed at full throttle FF on blocks

    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_VELOCITY_PID_KP = 0.02 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_VELOCITY_PID_KI = 0.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_VELOCITY_PID_KD = 0.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_VELOCITY_PID_KF = 0.00909 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0); // 100% control authority (on blocks)

    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_POSITION_PID_KP = 2.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_POSITION_PID_KI = 0.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_POSITION_PID_KD = 0.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_POSITION_PID_KF = 0.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);

    // DRIVE COMP
    public static final double SDSDRIVETRAIN_DRIVE_MOTOR_COMP_VELOCITY_PID_KS = 88.0; // RPM ~104.0 was highest speed at full throttle FF on blocks

    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_COMP_VELOCITY_PID_KP = 0.02 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_COMP_VELOCITY_PID_KI = 0.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_COMP_VELOCITY_PID_KD = 0.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_COMP_VELOCITY_PID_KF = 0.00961538 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0); // 100% control authority (on blocks)

    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_COMP_POSITION_PID_KP = 2.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_COMP_POSITION_PID_KI = 0.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_COMP_POSITION_PID_KD = 0.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_COMP_POSITION_PID_KF = 0.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);

    public static final double SDSDRIVETRAIN_OMEGA_POSITION_PID_KP = 0.1;
    public static final double SDSDRIVETRAIN_OMEGA_POSITION_PID_KI = 0.0;
    public static final double SDSDRIVETRAIN_OMEGA_POSITION_PID_KD = 0.0;
    public static final double SDSDRIVETRAIN_OMEGA_POSITION_PID_KF = 0.0;
    public static final double SDSDRIVETRAIN_OMEGA_POSITION_PID_KS = 1.0;
    public static final double SDSDRIVETRAIN_OMEGA_MAX_OUTPUT = 5.0;
    public static final double SDSDRIVETRAIN_OMEGA_MIN_OUTPUT = -5.0;

    public static final double SDSDRIVETRAIN_PATH_OMEGA_POSITION_PID_KP = 0.1;
    public static final double SDSDRIVETRAIN_PATH_OMEGA_POSITION_PID_KI = 0.0;
    public static final double SDSDRIVETRAIN_PATH_OMEGA_POSITION_PID_KD = 0.0;
    public static final double SDSDRIVETRAIN_PATH_OMEGA_POSITION_PID_KF = 0.0;
    public static final double SDSDRIVETRAIN_PATH_OMEGA_POSITION_PID_KS = 1.0;
    public static final double SDSDRIVETRAIN_PATH_OMEGA_MAX_OUTPUT = 4.0;
    public static final double SDSDRIVETRAIN_PATH_OMEGA_MIN_OUTPUT = -4.0;

    public static final double SDSDRIVETRAIN_PATH_X_POSITION_PID_KP = 1.0;
    public static final double SDSDRIVETRAIN_PATH_X_POSITION_PID_KI = 0.0;
    public static final double SDSDRIVETRAIN_PATH_X_POSITION_PID_KD = 0.0;
    public static final double SDSDRIVETRAIN_PATH_X_POSITION_PID_KF = 0.0;
    public static final double SDSDRIVETRAIN_PATH_X_POSITION_PID_KS = 1.0;
    public static final double SDSDRIVETRAIN_PATH_X_MAX_OUTPUT = 10.0;
    public static final double SDSDRIVETRAIN_PATH_X_MIN_OUTPUT = -10.0;

    public static final double SDSDRIVETRAIN_PATH_Y_POSITION_PID_KP = 1.0;
    public static final double SDSDRIVETRAIN_PATH_Y_POSITION_PID_KI = 0.0;
    public static final double SDSDRIVETRAIN_PATH_Y_POSITION_PID_KD = 0.0;
    public static final double SDSDRIVETRAIN_PATH_Y_POSITION_PID_KF = 0.0;
    public static final double SDSDRIVETRAIN_PATH_Y_POSITION_PID_KS = 1.0;
    public static final double SDSDRIVETRAIN_PATH_Y_MAX_OUTPUT = 10.0;
    public static final double SDSDRIVETRAIN_PATH_Y_MIN_OUTPUT = -10.0;

    public static final boolean SDSDRIVETRAIN_USE_OVERCURRENT_ADJUSTMENT = true;
    public static final double SDSDRIVETRAIN_OVERCURRENT_ADJUSTMENT = 0.75;
    public static final double SDSDRIVETRAIN_OVERCURRENT_HIGH_ADJUSTMENT = 0.5;

    public static final boolean SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED = true;
    public static final double SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION = 12.0;
    public static final boolean SDSDRIVETRAIN_DRIVE_SUPPLY_CURRENT_LIMITING_ENABLED = true;
    public static final double SDSDRIVETRAIN_DRIVE_SUPPLY_CURRENT_MAX = 35.0;
    public static final double SDSDRIVETRAIN_DRIVE_SUPPLY_TRIGGER_CURRENT = 35.0;
    public static final double SDSDRIVETRAIN_DRIVE_SUPPLY_TRIGGER_DURATION = 0.25;
    public static final boolean SDSDRIVETRAIN_DRIVE_STATOR_CURRENT_LIMITING_ENABLED = false;
    public static final double SDSDRIVETRAIN_DRIVE_STATOR_CURRENT_LIMIT = 80.0;

    public static final boolean SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED = true;
    public static final double SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION = 12.0;
    public static final boolean SDSDRIVETRAIN_STEER_SUPPLY_CURRENT_LIMITING_ENABLED = true;
    public static final double SDSDRIVETRAIN_STEER_SUPPLY_CURRENT_MAX = 20.0;
    public static final double SDSDRIVETRAIN_STEER_SUPPLY_TRIGGER_CURRENT = 30.0;
    public static final double SDSDRIVETRAIN_STEER_SUPPLY_TRIGGER_DURATION = 0.1;
    public static final boolean SDSDRIVETRAIN_STEER_STATOR_CURRENT_LIMITING_ENABLED = false;
    public static final double SDSDRIVETRAIN_STEER_STATOR_CURRENT_LIMIT = 80.0;

    public static final int SDSDRIVETRAIN_FEEDBACK_UPDATE_RATE_HZ = 100;
    public static final int SDSDRIVETRAIN_ERROR_UPDATE_RATE_HZ = 10;

    public static final boolean SDSDRIVETRAIN_SKIP_ANGLE_ON_ZERO_VELOCITY = true;
    public static final double SDSDRIVETRAIN_SKIP_ANGLE_ON_ZERO_DELTA = 0.001;
    public static final double SDSDRIVETRAIN_SKIP_OMEGA_ON_ZERO_DELTA = 0.36;

    public static final double SDSDRIVETRAIN_EXPONENTIAL = 2.0;
    public static final double SDSDRIVETRAIN_DEAD_ZONE_TURN = 0.1;
    public static final double SDSDRIVETRAIN_DEAD_ZONE_VELOCITY_X = 0.1;
    public static final double SDSDRIVETRAIN_DEAD_ZONE_VELOCITY_Y = 0.1;
    public static final double SDSDRIVETRAIN_DEAD_ZONE_TRIGGER_AB = 0.1;

    public static final double SDSDRIVETRAIN_ROTATION_A_MULTIPLIER = HardwareConstants.SDSDRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE / 2.0;
    public static final double SDSDRIVETRAIN_ROTATION_B_MULTIPLIER = HardwareConstants.SDSDRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE / 2.0;

    public static final double SDSDRIVETRAIN_MAX_VELOCITY = TuningConstants.SDSDRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KS * HardwareConstants.SDSDRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND; // max velocity in inches per second
    public static final double SDSDRIVETRAIN_SLOW_MODE_MAX_VELOCITY = 0.3 * TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY; // max velocity in inches per second
    public static final double SDSDRIVETRAIN_VELOCITY_TO_PERCENTAGE = 1.0 / TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY;
    public static final double SDSDRIVETRAIN_TURN_GOAL_VELOCITY = 10.0; // degrees per second for turn goal
    public static final double SDSDRIVETRAIN_TURN_SCALE = 1.6 * Math.PI; // radians per second
    public static final double SDSDRIVETRAIN_SLOW_MODE_TURN_SCALE = 0.3 * TuningConstants.SDSDRIVETRAIN_TURN_SCALE; // radians per second
    public static final double SDSDRIVETRAIN_STATIONARY_VELOCITY = 0.1;
    public static final double SDSDRIVETRAIN_TURN_APPROXIMATION_STATIONARY = 2.0; // number of degrees off at which point we give up trying to face an angle when uncommanded
    public static final double SDSDRIVETRAIN_TURN_APPROXIMATION = 1.0; // number of degrees off at which point we give up trying to face an angle when uncommanded
    public static final double SDSDRIVETRAIN_MAX_MODULE_PATH_VELOCITY = 0.85 * TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY; // up to x% of our max controllable speed
    public static final double SDSDRIVETRAIN_MAX_PATH_TURN_VELOCITY = 180.0; // in degrees per second
    public static final double SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY = 0.60 * TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY; // in inches per second
    public static final double SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION = 0.60 * TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY; // in inches per second per second
    public static final double SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY = 0.80 * TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY; // in inches per second
    public static final double SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION = 0.80 * TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY; // in inches per second per second
    public static final double SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY = 0.60 * TuningConstants.SDSDRIVETRAIN_MAX_PATH_TURN_VELOCITY; // in degrees per second
    public static final double SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION = 0.75 * TuningConstants.SDSDRIVETRAIN_MAX_PATH_TURN_VELOCITY; // in degrees per second per second
    public static final double SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY = TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY / 1.4; // in inches per second
    public static final double SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION = TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION / 1.25; // in inches per second per second
    public static final double SDSDRIVETRAIN_LOW_PATH_TRANSLATIONAL_VELOCITY = TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY / 2.0; // in inches per second
    public static final double SDSDRIVETRAIN_LOW_PATH_TRANSLATIONAL_ACCELERATION = TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION / 2.0; // in inches per second per second

    //================================================== REV DriveTrain ==============================================================

    public static final boolean REVDRIVETRAIN_STEER_MOTORS_USE_TRAPEZOIDAL_MOTION_PROFILE = true;
    public static final boolean REVDRIVETRAIN_USE_POSE_ESTIMATION_INVERSE_CORRECTION = false;
    public static final double REVDRIVETRAIN_POSE_ESTIMATION_INVERSE_TWIST_CORRECTION_TIMESTEP = 0.02;

    public static final boolean REVDRIVETRAIN_USE_ODOMETRY = true;
    public static final boolean REVDRIVETRAIN_RESET_ON_ROBOT_START = true;
    public static final boolean REVDRIVETRAIN_FIELD_ORIENTED_ON_ROBOT_START = true;
    public static final boolean REVDRIVETRAIN_MAINTAIN_ORIENTATION_ON_ROBOT_START = true;

    public static final double REVDRIVETRAIN_STEER_MOTOR1_ABSOLUTE_OFFSET = 0.0;
    public static final double REVDRIVETRAIN_STEER_MOTOR2_ABSOLUTE_OFFSET = 0.0;
    public static final double REVDRIVETRAIN_STEER_MOTOR3_ABSOLUTE_OFFSET = 0.0;
    public static final double REVDRIVETRAIN_STEER_MOTOR4_ABSOLUTE_OFFSET = 0.0;

    public static final boolean REVDRIVETRAIN_USE_TRANSLATIONAL_RATE_LIMITING = true;
    public static final double REVDRIVETRAIN_TRANSLATIONAL_VELOCITY_MAX_NEGATIVE_RATE = -3.0 * TuningConstants.REVDRIVETRAIN_MAX_VELOCITY;
    public static final double REVDRIVETRAIN_TRANSLATIONAL_VELOCITY_MAX_POSITIVE_RATE = 3.0 * TuningConstants.REVDRIVETRAIN_MAX_VELOCITY;
    public static final boolean REVDRIVETRAIN_USE_ROTATIONAL_RATE_LIMITING = true;
    public static final double REVDRIVETRAIN_ROTATIONAL_VELOCITY_MAX_NEGATIVE_RATE = -4.0 * TuningConstants.REVDRIVETRAIN_TURN_SCALE;
    public static final double REVDRIVETRAIN_ROTATIONAL_VELOCITY_MAX_POSITIVE_RATE = 4.0 * TuningConstants.REVDRIVETRAIN_TURN_SCALE;

    // Position PID (angle) per-module
    public static final double REVDRIVETRAIN_STEER_MOTOR_POSITION_PID_KS = HardwareConstants.REVDRIVETRAIN_STEER_TICKS_PER_DEGREE;

    public static final boolean REVDRIVETRAIN_STEER_MOTORS_POSITION_PID_WRAPPING_ENABLED = true;
    public static final double REVDRIVETRAIN_STEER_MOTORS_POSITION_PID_WRAPPING_MIN = 0.0;
    public static final double REVDRIVETRAIN_STEER_MOTORS_POSITION_PID_WRAPPING_MAX = 360.0;

    public static final double REVDRIVETRAIN_STEER_MOTORS_POSITION_PID_KP = 0.0166; // try to go full speed for a 60deg difference (or .016 power for a 1deg error)
    public static final double REVDRIVETRAIN_STEER_MOTORS_POSITION_PID_KI = 0.0;
    public static final double REVDRIVETRAIN_STEER_MOTORS_POSITION_PID_KD = 0.0;
    public static final double REVDRIVETRAIN_STEER_MOTORS_POSITION_PID_KF = 0.0;

    // Offboard TrapezoidalMotionProfile system...
    public static final double REVDRIVETRAIN_STEER_MOTORS_TMP_PID_KP = 0.0166;
    public static final double REVDRIVETRAIN_STEER_MOTORS_TMP_PID_KI = 0.0;
    public static final double REVDRIVETRAIN_STEER_MOTORS_TMP_PID_KD = 0.0;
    public static final double REVDRIVETRAIN_STEER_MOTORS_TMP_PID_KF = 0.0;
    public static final double REVDRIVETRAIN_STEER_MOTORS_TMP_PID_CRUISE_VELOC = 1440.0; // deg/sec
    public static final double REVDRIVETRAIN_STEER_MOTORS_TMP_PID_ACCEL = 360 * 32; // 4 * 360; // deg/sec/sec

    // Velocity PID (drive) per-module
    public static final double REVDRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KS = 200.0; // 200.0 was highest speed at full throttle FF on blocks. this is inches/second

    public static final double REVDRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KP = 0.006;
    public static final double REVDRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KI = 0.0;
    public static final double REVDRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KD = 0.0;
    public static final double REVDRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KF = 0.005;

    public static final double REVDRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KP = 1.0;
    public static final double REVDRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KI = 0.0;
    public static final double REVDRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KD = 0.0;
    public static final double REVDRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KF = 0.0;

    public static final double REVDRIVETRAIN_OMEGA_POSITION_PID_KP = 0.1;
    public static final double REVDRIVETRAIN_OMEGA_POSITION_PID_KI = 0.0;
    public static final double REVDRIVETRAIN_OMEGA_POSITION_PID_KD = 0.0;
    public static final double REVDRIVETRAIN_OMEGA_POSITION_PID_KF = 0.0;
    public static final double REVDRIVETRAIN_OMEGA_POSITION_PID_KS = 1.0;
    public static final double REVDRIVETRAIN_OMEGA_MAX_OUTPUT = 5.0;
    public static final double REVDRIVETRAIN_OMEGA_MIN_OUTPUT = -5.0;

    public static final double REVDRIVETRAIN_PATH_OMEGA_POSITION_PID_KP = 0.1;
    public static final double REVDRIVETRAIN_PATH_OMEGA_POSITION_PID_KI = 0.0;
    public static final double REVDRIVETRAIN_PATH_OMEGA_POSITION_PID_KD = 0.0;
    public static final double REVDRIVETRAIN_PATH_OMEGA_POSITION_PID_KF = 0.0;
    public static final double REVDRIVETRAIN_PATH_OMEGA_POSITION_PID_KS = 1.0;
    public static final double REVDRIVETRAIN_PATH_OMEGA_MAX_OUTPUT = 4.0;
    public static final double REVDRIVETRAIN_PATH_OMEGA_MIN_OUTPUT = -4.0;

    public static final double REVDRIVETRAIN_PATH_X_POSITION_PID_KP = 0.0;
    public static final double REVDRIVETRAIN_PATH_X_POSITION_PID_KI = 0.0;
    public static final double REVDRIVETRAIN_PATH_X_POSITION_PID_KD = 0.0;
    public static final double REVDRIVETRAIN_PATH_X_POSITION_PID_KF = 0.0;
    public static final double REVDRIVETRAIN_PATH_X_POSITION_PID_KS = 1.0;
    public static final double REVDRIVETRAIN_PATH_X_MAX_OUTPUT = 10.0;
    public static final double REVDRIVETRAIN_PATH_X_MIN_OUTPUT = -10.0;

    public static final double REVDRIVETRAIN_PATH_Y_POSITION_PID_KP = 0.0;
    public static final double REVDRIVETRAIN_PATH_Y_POSITION_PID_KI = 0.0;
    public static final double REVDRIVETRAIN_PATH_Y_POSITION_PID_KD = 0.0;
    public static final double REVDRIVETRAIN_PATH_Y_POSITION_PID_KF = 0.0;
    public static final double REVDRIVETRAIN_PATH_Y_POSITION_PID_KS = 1.0;
    public static final double REVDRIVETRAIN_PATH_Y_MAX_OUTPUT = 10.0;
    public static final double REVDRIVETRAIN_PATH_Y_MIN_OUTPUT = -10.0;

    public static final boolean REVDRIVETRAIN_USE_OVERCURRENT_ADJUSTMENT = true;
    public static final double REVDRIVETRAIN_OVERCURRENT_ADJUSTMENT = 0.75;
    public static final double REVDRIVETRAIN_OVERCURRENT_HIGH_ADJUSTMENT = 0.5;

    public static final int REVDRIVETRAIN_DRIVE_CURRENT_STALL_LIMIT = 50;
    public static final int REVDRIVETRAIN_DRIVE_CURRENT_FREE_LIMIT = 0;
    public static final int REVDRIVETRAIN_DRIVE_CURRENT_LIMIT_RPM = 20000;

    public static final int REVDRIVETRAIN_STEER_CURRENT_STALL_LIMIT = 20;
    public static final int REVDRIVETRAIN_STEER_CURRENT_FREE_LIMIT = 0;
    public static final int REVDRIVETRAIN_STEER_CURRENT_LIMIT_RPM = 20000;

    public static final boolean REVDRIVETRAIN_SKIP_ANGLE_ON_ZERO_VELOCITY = true;
    public static final double REVDRIVETRAIN_SKIP_ANGLE_ON_ZERO_DELTA = 0.001;
    public static final double REVDRIVETRAIN_SKIP_OMEGA_ON_ZERO_DELTA = 0.36;

    public static final double REVDRIVETRAIN_EXPONENTIAL = 2.0;
    public static final double REVDRIVETRAIN_DEAD_ZONE_TURN = 0.1;
    public static final double REVDRIVETRAIN_DEAD_ZONE_VELOCITY_X = 0.1;
    public static final double REVDRIVETRAIN_DEAD_ZONE_VELOCITY_Y = 0.1;
    public static final double REVDRIVETRAIN_DEAD_ZONE_TRIGGER_AB = 0.1;

    public static final double REVDRIVETRAIN_ROTATION_A_MULTIPLIER = HardwareConstants.REVDRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE / 2.0;
    public static final double REVDRIVETRAIN_ROTATION_B_MULTIPLIER = HardwareConstants.REVDRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE / 2.0;

    public static final double REVDRIVETRAIN_MAX_VELOCITY = TuningConstants.REVDRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KS; // max velocity in inches per second
    public static final double REVDRIVETRAIN_SLOW_MODE_MAX_VELOCITY = 0.3 * TuningConstants.REVDRIVETRAIN_MAX_VELOCITY; // max velocity in inches per second
    public static final double REVDRIVETRAIN_VELOCITY_TO_PERCENTAGE = 1.0 / TuningConstants.REVDRIVETRAIN_MAX_VELOCITY;
    public static final double REVDRIVETRAIN_TURN_GOAL_VELOCITY = 10.0; // degrees per second for turn goal
    public static final double REVDRIVETRAIN_TURN_SCALE = 1.6 * Math.PI; // radians per second
    public static final double REVDRIVETRAIN_SLOW_MODE_TURN_SCALE = 0.3 * TuningConstants.REVDRIVETRAIN_TURN_SCALE; // radians per second
    public static final double REVDRIVETRAIN_STATIONARY_VELOCITY = 0.1;
    public static final double REVDRIVETRAIN_TURN_APPROXIMATION_STATIONARY = 2.0; // number of degrees off at which point we give up trying to face an angle when uncommanded
    public static final double REVDRIVETRAIN_TURN_APPROXIMATION = 1.0; // number of degrees off at which point we give up trying to face an angle when uncommanded
    public static final double REVDRIVETRAIN_MAX_MODULE_PATH_VELOCITY = 0.85 * TuningConstants.REVDRIVETRAIN_MAX_VELOCITY; // up to x% of our max controllable speed
    public static final double REVDRIVETRAIN_MAX_PATH_TURN_VELOCITY = 180.0; // in degrees per second
    public static final double REVDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY = 0.60 * TuningConstants.REVDRIVETRAIN_MAX_VELOCITY; // in inches per second
    public static final double REVDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY = 0.75 * TuningConstants.REVDRIVETRAIN_MAX_VELOCITY; // in inches per second
    public static final double REVDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY = 0.25 * TuningConstants.REVDRIVETRAIN_MAX_PATH_TURN_VELOCITY; // in degrees per second
    public static final double REVDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION = 0.5 * TuningConstants.REVDRIVETRAIN_MAX_PATH_TURN_VELOCITY; // in degrees per second per second
    public static final double REVDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION = 0.75 * TuningConstants.REVDRIVETRAIN_MAX_VELOCITY; // in inches per second per second
    public static final double REVDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY = TuningConstants.REVDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY / 1.4; // in inches per second
    public static final double REVDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION = TuningConstants.REVDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION / 1.25; // in inches per second per second
    public static final double REVDRIVETRAIN_LOW_PATH_TRANSLATIONAL_VELOCITY = TuningConstants.REVDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY / 2.0; // in inches per second
    public static final double REVDRIVETRAIN_LOW_PATH_TRANSLATIONAL_ACCELERATION = TuningConstants.REVDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION / 2.0; // in inches per second per second

    //============================================================ AUTONOMOUS POSITIONS ======================================

    public static final double RevStartPositionX = 0.0;
    public static final double RevStartPositionY = 0.0;

    //======================================================== Electrolite Arm Constants ===================================

    public static final boolean WRIST_MOTOR_USE_SMART_MOTION = false;
    public static final boolean WRIST_MOTOR_INVERT_SENSOR = true;
    public static final boolean WRIST_MOTOR_INVERT_OUTPUT = false;
    public static final boolean INTAKE_MOTOR_INVERT_OUTPUT = false;

    public static final double WRIST_MOTOR_POSITION_PID_KP = 0.015;
    public static final double WRIST_MOTOR_POSITION_PID_KI = 0;
    public static final double WRIST_MOTOR_POSITION_PID_KD = 0;
    public static final double WRIST_MOTOR_POSITION_PID_KF = 0.0;
    public static final double WRIST_MOTOR_SM_PID_KP = 0.02;
    public static final double WRIST_MOTOR_SM_PID_KI = 0.0;
    public static final double WRIST_MOTOR_SM_PID_KD = 0.0;
    public static final double WRIST_MOTOR_SM_PID_KF = 0.02;
    public static final int WRIST_MOTOR_SM_IZONE = 0;
    public static final int WRIST_MOTOR_SM_PID_CRUISE_VELOC = 10;
    public static final int WRIST_MOTOR_SM_PID_ACCEL = 15;

    public static final double WRIST_INTAKE_IN_POWER = 0.4;
    public static final double WRIST_INTAKE_OUT_POWER = -0.95;
    public static final double WRIST_INTAKE_OUT_MEDIUM_POWER = -0.6;
    public static final double WRIST_INTAKE_OUT_SLOW_POWER = -0.4;
    public static final double WRIST_INTAKE_OUT_SUPER_SLOW_POWER = -0.2;

    public static final double INTAKE_MOTOR_VELOCITY_PID_KP = 0.015;
    public static final double INTAKE_MOTOR_VELOCITY_PID_KI = 0;
    public static final double INTAKE_MOTOR_VELOCITY_PID_KD = 0;
    public static final double INTAKE_MOTOR_VELOCITY_PID_KF = 0.0;
    public static final double INTAKE_MIN_THRESHOLD = 0.5;

    public static final boolean INTAKE_MOTOR_USE_VELOCITY_CONTROL = false; 

    public static final double WRIST_INPUT_TO_TICK_ADJUSTMENT = 25.0;
    public static final double WRIST_MIN_THRESHOLD = 5.0;
    public static final double WRIST_DEADZONE = 0.1;

    public static final double LOW_CUBE_DROP_POSITION = 230.0;
    public static final double MID_CUBE_DROP_POSITION = 280.0;
    public static final double HIGH_CUBE_DROP_POSITION = 330.0;
    public static final double STOWED_POSITION = 350.0;
    public static final double GROUND_CUBE_PICKUP = 207.0;
    public static final double SUBSTATION_CUBE_PICKUP = 320.0;

    public static final boolean WRIST_MOTOR_POSITION_PID_WRAPPING_ENABLED = true;
    public static final double WRIST_MOTOR_POSITION_PID_WRAPPING_MIN = 0.0;
    public static final double WRIST_MOTOR_POSITION_PID_WRAPPING_MAX = 360.0;

    // Wrist stall prevention
    public static final boolean WRIST_STALL_PROTECTION_ENABLED = false;
    public static final double WRIST_STALLED_CURRENT_THRESHOLD = 2.5; // amount of current being "used" by the wrist motor (despite not moving according to the encoders) to be considered stalled
    public static final double BATTERY_AVERAGE_EXPECTED_VOLTAGE = 12.0; // expected voltage of battery
    public static final double WRIST_STALLED_POWER_THRESHOLD = TuningConstants.WRIST_STALLED_CURRENT_THRESHOLD * TuningConstants.BATTERY_AVERAGE_EXPECTED_VOLTAGE; // amount of power that can be "used" by the linear actuators to be considered stalled
    public static final double WRIST_STALLED_VELOCITY_THRESHOLD = 0.05; // .05 rot/sec is very slow

    // Power sampling for wrist
    public static final double WRIST_POWER_TRACKING_DURATION = 0.25; // duration of time to keep track of the average current
    public static final double WRIST_POWER_SAMPLES_PER_LOOP = 1.0;
    public static final double WRIST_POWER_SAMPLES_PER_SECOND = TuningConstants.LOOPS_PER_SECOND * TuningConstants.WRIST_POWER_SAMPLES_PER_LOOP;
    public static final double WRIST_NOT_MOVING_POWER_THRESHOLD = 0.25; // amount of power being "used" by the wrist motorr to be considered "not moving"

    // Velocity sampling for wrist
    public static final double WRIST_VELOCITY_TRACKING_DURATION = TuningConstants.WRIST_POWER_TRACKING_DURATION; // should match the power tracking
    public static final double WRIST_VELOCITY_SAMPLES_PER_LOOP = TuningConstants.WRIST_POWER_SAMPLES_PER_LOOP; // should match the power tracking
    public static final double WRIST_VELOCITY_SAMPLES_PER_SECOND = TuningConstants.WRIST_POWER_SAMPLES_PER_SECOND;

    // Intake stall prevention
    public static final boolean INTAKE_STALL_PROTECTION_ENABLED = true;
    public static final double INTAKE_STALLED_CURRENT_THRESHOLD = 5.0; // amount of current being "used" by the wrist motor (despite not moving according to the encoders) to be considered stalled
    public static final double INTAKE_STALLED_POWER_THRESHOLD = TuningConstants.INTAKE_STALLED_CURRENT_THRESHOLD * TuningConstants.BATTERY_AVERAGE_EXPECTED_VOLTAGE; // amount of power that can be "used" by the linear actuators to be considered stalled
    public static final double INTAKE_STALLED_VELOCITY_THRESHOLD = 1000.0; // 1000 units would be very slow

    // Power sampling for intake
    public static final double INTAKE_POWER_TRACKING_DURATION = 0.25; // duration of time to keep track of the average current
    public static final double INTAKE_POWER_SAMPLES_PER_LOOP = 1.0;
    public static final double INTAKE_POWER_SAMPLES_PER_SECOND = TuningConstants.LOOPS_PER_SECOND * TuningConstants.INTAKE_POWER_SAMPLES_PER_LOOP;
    public static final double INTAKE_NOT_MOVING_POWER_THRESHOLD = 0.25; // amount of power being "used" by the intake motorr to be considered "not moving"

    // Velocity sampling for intake
    public static final double INTAKE_VELOCITY_TRACKING_DURATION = TuningConstants.INTAKE_POWER_TRACKING_DURATION; // should match the power tracking
    public static final double INTAKE_VELOCITY_SAMPLES_PER_LOOP = TuningConstants.INTAKE_POWER_SAMPLES_PER_LOOP; // should match the power tracking
    public static final double INTAKE_VELOCITY_SAMPLES_PER_SECOND = TuningConstants.INTAKE_POWER_SAMPLES_PER_SECOND;
}
