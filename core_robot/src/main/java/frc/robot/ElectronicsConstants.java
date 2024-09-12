package frc.robot;

import frc.lib.robotprovider.PneumaticsModuleType;
import frc.lib.robotprovider.PowerDistributionModuleType;

/**
 * All constants describing how the electronics are plugged together.
 * 
 * @author Will
 * 
 */
public class ElectronicsConstants
{
    // We expect the following to be true.  Change INVERT_*_AXIS to true if any of the following are not met:
    // 1. forwards/up on a joystick is positive, backwards/down is negative.
    // 2. right on a joystick is positive, left on a joystick is negative.
    // 3. pressed on a trigger is positive, released is negative/zero.
    public static final boolean INVERT_XBONE_LEFT_X_AXIS = false;
    public static final boolean INVERT_XBONE_RIGHT_X_AXIS = false;
    public static final boolean INVERT_XBONE_LEFT_Y_AXIS = true;
    public static final boolean INVERT_XBONE_RIGHT_Y_AXIS = true;
    public static final boolean INVERT_XBONE_LEFT_TRIGGER = false;
    public static final boolean INVERT_XBONE_RIGHT_TRIGGER = false;

    public static final boolean INVERT_PS4_LEFT_X_AXIS = false;
    public static final boolean INVERT_PS4_RIGHT_X_AXIS = false;
    public static final boolean INVERT_PS4_LEFT_Y_AXIS = true;
    public static final boolean INVERT_PS4_RIGHT_Y_AXIS = true;
    public static final boolean INVERT_PS4_LEFT_TRIGGER = false;
    public static final boolean INVERT_PS4_RIGHT_TRIGGER = false;

    public static final boolean INVERT_THROTTLE_AXIS = true;
    public static final boolean INVERT_TRIGGER_AXIS = false;

    public static final int POWER_DISTRIBUTION_CAN_ID = 0;
    public static final PowerDistributionModuleType POWER_DISTRIBUTION_TYPE = PowerDistributionModuleType.PowerDistributionPanel;

    public static final double REV_THROUGHBORE_ENCODER_DUTY_CYCLE_MIN = 1.0 / 1024.0;
    public static final double REV_THROUGHBORE_ENCODER_DUTY_CYCLE_MAX = 1023.0 / 1024.0;

    public static final String CANIVORE_NAME = "CANIVORE1"; // Module A

    public static final int PNEUMATICS_MODULE_A = 1; // Module A
    public static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE_A = PneumaticsModuleType.PneumaticsHub; // Module A

    public static final int PNEUMATICS_MODULE_B = 2; // Module B
    public static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE_B = PneumaticsModuleType.PneumaticsHub; // Module B

    public static final boolean PNEUMATICS_USE_HYBRID = false;
    public static final boolean PNEUMATICS_USE_ANALOG = false;
    public static final double PNEUMATICS_MIN_PSI = 110.0;
    public static final double PNEUMATICS_MAX_PSI = 120.0;

    //================================================== IMU ==============================================================

    public static final int PIGEON_IMU_CAN_ID = 0;

    //================================================== Indicator Lights ==============================================================

    public static final int INDICATOR_LIGHT_CANDLE_CAN_ID = 55;

    //================================================== SDSDriveTrain ==============================================================

    public static final int SDSDRIVETRAIN_DRIVE_MOTOR_1_CAN_ID = 1;
    public static final int SDSDRIVETRAIN_STEER_MOTOR_1_CAN_ID = 2;
    public static final int SDSDRIVETRAIN_DRIVE_MOTOR_2_CAN_ID = 3;
    public static final int SDSDRIVETRAIN_STEER_MOTOR_2_CAN_ID = 4;
    public static final int SDSDRIVETRAIN_DRIVE_MOTOR_3_CAN_ID = 5;
    public static final int SDSDRIVETRAIN_STEER_MOTOR_3_CAN_ID = 6;
    public static final int SDSDRIVETRAIN_DRIVE_MOTOR_4_CAN_ID = 7;
    public static final int SDSDRIVETRAIN_STEER_MOTOR_4_CAN_ID = 8;

    public static final int SDSDRIVETRAIN_ABSOLUTE_ENCODER_1_CAN_ID = 1;
    public static final int SDSDRIVETRAIN_ABSOLUTE_ENCODER_2_CAN_ID = 2;
    public static final int SDSDRIVETRAIN_ABSOLUTE_ENCODER_3_CAN_ID = 3;
    public static final int SDSDRIVETRAIN_ABSOLUTE_ENCODER_4_CAN_ID = 4;

    //================================================== RevDriveTrain ==============================================================

    public static final int REVDRIVETRAIN_DRIVE_MOTOR_1_CAN_ID = 1;
    public static final int REVDRIVETRAIN_STEER_MOTOR_1_CAN_ID = 2;
    public static final int REVDRIVETRAIN_DRIVE_MOTOR_2_CAN_ID = 3;
    public static final int REVDRIVETRAIN_STEER_MOTOR_2_CAN_ID = 4;
    public static final int REVDRIVETRAIN_DRIVE_MOTOR_3_CAN_ID = 5;
    public static final int REVDRIVETRAIN_STEER_MOTOR_3_CAN_ID = 6;
    public static final int REVDRIVETRAIN_DRIVE_MOTOR_4_CAN_ID = 7;
    public static final int REVDRIVETRAIN_STEER_MOTOR_4_CAN_ID = 8;

    //================================================== Wrist Intake ==============================================================

    public static final int WRIST_MOTOR_CAN_ID = 30;
    public static final int INTAKE_MOTOR_CAN_ID = 40;
    // intake wheels Rev can ID is 30
    // intake wrist Rev CAN ID is 40
    public static final int WRIST_PDH_CHANNEL = 4;
    public static final int WRIST_INTAKE_PDH_CHANNEL = 5;
}

