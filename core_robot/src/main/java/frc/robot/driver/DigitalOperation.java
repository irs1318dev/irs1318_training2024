package frc.robot.driver;

import frc.lib.driver.IOperation;

public enum DigitalOperation implements IOperation
{
    PositionResetFieldOrientation,
    PositionResetRobotLevel,

    // Driver interaction operations
    ForceLightDriverRumble,

    // Vision operations:
    VisionForceDisable,
    VisionEnableStream,
    VisionFindSpecificAprilTag,
    VisionFindAnyAprilTag,
    VisionFindAbsolutePosition,

    // Compressor operations:
    CompressorForceDisable,

    // DriveTrain operations:
    DriveTrainSlowMode,
    DriveTrainPathMode,
    DriveTrainSteerMode,
    DriveTrainMaintainPositionMode,
    DriveTrainReset,
    DriveTrainEnableFieldOrientation,
    DriveTrainDisableFieldOrientation,
    DriveTrainUseRobotOrientation,
    DriveTrainEnableMaintainDirectionMode,
    DriveTrainDisableMaintainDirectionMode,
    DriveTrainResetXYPosition,
    DriveTrainIgnoreSlewRateLimitingMode,

    // Wrist operations:
    WristEnableSimpleMode, // make a button
    WristDisableSimpleMode, // make a button
    IntakeIn, // make a button
    IntakeOutMedium,
    IntakeOut, // make a button
    IntakeOutSlow,
    IntakeOutSuperSlow,
    IntakeEnableSimpleMode,
    IntakeDisableSimpleMode,
}
