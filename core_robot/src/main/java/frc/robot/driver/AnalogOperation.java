package frc.robot.driver;

import frc.lib.driver.IOperation;

public enum AnalogOperation implements IOperation
{
    PositionStartingAngle,

    // DriveTrain operations:          (perspective)
    DriveTrainMoveForward,          // driver-relative
    DriveTrainMoveRight,            // driver-relative
    DriveTrainTurnAngleGoal,        // driver-relative
    DriveTrainSpinLeft,             // driver-relative
    DriveTrainSpinRight,            // driver-relative
    DriveTrainRotationA,            // robot-relative
    DriveTrainRotationB,            // robot-relative
    DriveTrainPathXGoal,            // absolute
    DriveTrainPathYGoal,            // absolute
    DriveTrainPathXVelocityGoal,    // absolute
    DriveTrainPathYVelocityGoal,    // absolute
    DriveTrainPathAngleGoal,        // absolute
    DriveTrainPathAngleVelocityGoal,// absolute
    DriveTrainPositionSteer1,       // robot-relative
    DriveTrainPositionSteer2,       // robot-relative
    DriveTrainPositionSteer3,       // robot-relative
    DriveTrainPositionSteer4,       // robot-relative
    DriveTrainPositionDrive1,       // robot-relative
    DriveTrainPositionDrive2,       // robot-relative
    DriveTrainPositionDrive3,       // robot-relative
    DriveTrainPositionDrive4,       // robot-relative
    DriveTrainStartingXPosition,    // (absolute)
    DriveTrainStartingYPosition,    // (absolute)

    // Wrist operations:
    WristAngleAdjustment,
    WristSetAngle,
    IntakeMotorVelocityGoal,
}
