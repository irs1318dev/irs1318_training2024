package frc.robot.driver;

import frc.lib.driver.IOperation;

public enum MacroOperation implements IOperation
{
    AutonomousRoutine,

    // DriveTrain operations:
    PIDLightBrake,
    PIDHeavyBrake,
    FaceForward,
    FaceBackward,
    FaceLeft,
    FaceRight,
    FaceSomething,

    // Vision operations

    // Path testing:
    FollowPathTest1,
    FollowPathTest2,
    FollowPathTest3,
    FollowPathTest4,

    // Wrist operations:
    LowCubeDrop,
    MidCubeDrop,
    HighCubeDrop,
    WristStowed,
    SubstationPickup,
    GroundPickup, 
    Test2024,
    Test2024_1,
    Test2024_2,
    Test2024_3,
    Test2024_4,
    Test2024_5,
    Test2024_6,
    Test2024_7,
    Test2024_8
}
