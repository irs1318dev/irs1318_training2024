package frc.robot.driver;

import java.util.EnumSet;

import javax.inject.Singleton;

import frc.lib.driver.*;
import frc.lib.driver.buttons.*;
import frc.lib.driver.descriptions.*;
import frc.lib.helpers.Helpers;
import frc.robot.*;
import frc.robot.driver.controltasks.*;
import frc.robot.driver.controltasks.FieldOrientationTask.DesiredOrientation;
import frc.robot.driver.controltasks.FollowPathTask.Type;

@Singleton
public class ButtonMap implements IButtonMap
{
    private static ShiftDescription[] ShiftButtonSchema = new ShiftDescription[]
    {
        new ShiftDescription(
            Shift.DriverDebug,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_LEFT_BUTTON),
        new ShiftDescription(
            Shift.CodriverDebug,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_LEFT_BUTTON),
        new ShiftDescription(
            Shift.Test1Debug,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_LEFT_BUTTON),
        // new ShiftDescription(
        //     Shift.Test2Debug,
        //     UserInputDevice.Test2,
        //     UserInputDeviceButton.XBONE_LEFT_BUTTON),
    };

    static double framePreremetere = 22; //With bumpers
    static double halfFramePreremetere = framePreremetere / 2.0;

    public static AnalogOperationDescription[] AnalogOperationSchema = new AnalogOperationDescription[]
    {
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainMoveForward,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_LSY,
            ElectronicsConstants.INVERT_XBONE_LEFT_Y_AXIS,
            -TuningConstants.SDSDRIVETRAIN_DEAD_ZONE_VELOCITY_Y,
            TuningConstants.SDSDRIVETRAIN_DEAD_ZONE_VELOCITY_Y,
            1.0,
            TuningConstants.SDSDRIVETRAIN_EXPONENTIAL),
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainMoveRight,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_LSX,
            ElectronicsConstants.INVERT_XBONE_LEFT_X_AXIS,
            -TuningConstants.SDSDRIVETRAIN_DEAD_ZONE_VELOCITY_X,
            TuningConstants.SDSDRIVETRAIN_DEAD_ZONE_VELOCITY_X,
            1.0,
            TuningConstants.SDSDRIVETRAIN_EXPONENTIAL),
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainTurnAngleGoal,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_RSX,
            AnalogAxis.XBONE_RSY,
            EnumSet.noneOf(Shift.class),
            EnumSet.noneOf(Shift.class),
            !ElectronicsConstants.INVERT_XBONE_RIGHT_X_AXIS, // make left positive...
            ElectronicsConstants.INVERT_XBONE_RIGHT_Y_AXIS,
            0.0,
            TuningConstants.SDSDRIVETRAIN_SKIP_OMEGA_ON_ZERO_DELTA,
            true,
            TuningConstants.MAGIC_NULL_VALUE,
            (x, y) -> Helpers.atan2d(x, y)),
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainSpinLeft,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_LT,
            ElectronicsConstants.INVERT_XBONE_LEFT_TRIGGER, // turning left should be positive, as counter-clockwise is positive
            -TuningConstants.SDSDRIVETRAIN_DEAD_ZONE_TURN,
            TuningConstants.SDSDRIVETRAIN_DEAD_ZONE_TURN),
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainSpinRight,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_RT,
            !ElectronicsConstants.INVERT_XBONE_RIGHT_TRIGGER, // make left positive, as counter-clockwise is positive
            -TuningConstants.SDSDRIVETRAIN_DEAD_ZONE_TURN,
            TuningConstants.SDSDRIVETRAIN_DEAD_ZONE_TURN),

        // wrist Operation
        new AnalogOperationDescription(
            AnalogOperation.WristAngleAdjustment,
            UserInputDevice.Codriver,
            AnalogAxis.XBONE_LSY,
            ElectronicsConstants.INVERT_XBONE_LEFT_Y_AXIS,
            TuningConstants.WRIST_DEADZONE),

        new AnalogOperationDescription(
            AnalogOperation.IntakeMotorVelocityGoal,
            TuningConstants.MAGIC_NULL_VALUE),

        new AnalogOperationDescription(
            AnalogOperation.WristSetAngle,
            TuningConstants.MAGIC_NULL_VALUE),
    };

    public static DigitalOperationDescription[] DigitalOperationSchema = new DigitalOperationDescription[]
    {
        // driving operations
        new DigitalOperationDescription(
            DigitalOperation.PositionResetFieldOrientation,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_Y_BUTTON,
            EnumSet.noneOf(Shift.class),
            EnumSet.noneOf(Shift.class),
            ButtonType.Click),
        // new DigitalOperationDescription(
        //     DigitalOperation.DriveTrainReset,
        //     UserInputDevice.Driver,
        //     UserInputDeviceButton.XBONE_Y_BUTTON,
        //     EnumSet.of(Shift.DriverDebug),
        //     EnumSet.of(Shift.DriverDebug),
        //     ButtonType.Click),
        // new DigitalOperationDescription(
        //     DigitalOperation.DriveTrainEnableFieldOrientation,
        //     UserInputDevice.Driver,
        //     270,
        //     EnumSet.of(Shift.DriverDebug),
        //     EnumSet.noneOf(Shift.class),
        //     ButtonType.Click),
        // new DigitalOperationDescription(
        //     DigitalOperation.DriveTrainDisableFieldOrientation,
        //     UserInputDevice.Driver,
        //     270,
        //     EnumSet.of(Shift.DriverDebug),
        //     EnumSet.of(Shift.DriverDebug),
        //     ButtonType.Click),
        // new DigitalOperationDescription(
        //     DigitalOperation.DriveTrainEnableMaintainDirectionMode,
        //     UserInputDevice.Driver,
        //     90,
        //     EnumSet.of(Shift.DriverDebug),
        //     EnumSet.noneOf(Shift.class),
        //     ButtonType.Click),
        // new DigitalOperationDescription(
        //     DigitalOperation.DriveTrainDisableMaintainDirectionMode,
        //     UserInputDevice.Driver,
        //     90,
        //     EnumSet.of(Shift.DriverDebug),
        //     EnumSet.of(Shift.DriverDebug),
        //     ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.DriveTrainSlowMode,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_X_BUTTON,
            EnumSet.noneOf(Shift.class),
            EnumSet.noneOf(Shift.class),
            ButtonType.Simple),

        // Vision test operations:
        new DigitalOperationDescription(
            DigitalOperation.VisionFindAnyAprilTag,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_B_BUTTON,
            ButtonType.Toggle),
        new DigitalOperationDescription(
            DigitalOperation.VisionFindAbsolutePosition,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_X_BUTTON,
            ButtonType.Simple),

        // Wrist operations:
        new DigitalOperationDescription(
            DigitalOperation.IntakeIn,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_A_BUTTON,
            EnumSet.noneOf(Shift.class),
            EnumSet.noneOf(Shift.class),
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.IntakeOut,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_B_BUTTON,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.IntakeOutSlow,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_B_BUTTON,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.of(Shift.DriverDebug),
            ButtonType.Simple),

        new DigitalOperationDescription(
            DigitalOperation.WristEnableSimpleMode,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_X_BUTTON,
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.WristDisableSimpleMode,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_Y_BUTTON,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.IntakeOutSuperSlow,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_Y_BUTTON,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.of(Shift.CodriverDebug),
            ButtonType.Simple),
    };

    public static MacroOperationDescription[] MacroSchema = new MacroOperationDescription[]
    {
        // driving macros
        // new MacroOperationDescription(
        //     MacroOperation.PIDLightBrake,
        //     UserInputDevice.Driver,
        //     UserInputDeviceButton.XBONE_LEFT_STICK_BUTTON,
        //     EnumSet.of(Shift.DriverDebug),
        //     EnumSet.noneOf(Shift.class),
        //     ButtonType.Simple,
        //     () -> new PIDBrakeTask(false),
        //     new IOperation[]
        //     {
        //         AnalogOperation.DriveTrainMoveForward,
        //         AnalogOperation.DriveTrainMoveRight,
        //         AnalogOperation.DriveTrainTurnAngleGoal,
        //         AnalogOperation.DriveTrainSpinLeft,
        //         AnalogOperation.DriveTrainSpinRight,
        //         AnalogOperation.DriveTrainRotationA,
        //         AnalogOperation.DriveTrainRotationB,
        //         AnalogOperation.DriveTrainPathXGoal,
        //         AnalogOperation.DriveTrainPathYGoal,
        //         AnalogOperation.DriveTrainPathXVelocityGoal,
        //         AnalogOperation.DriveTrainPathYVelocityGoal,
        //         AnalogOperation.DriveTrainPathAngleVelocityGoal,
        //         AnalogOperation.DriveTrainPositionDrive1,
        //         AnalogOperation.DriveTrainPositionDrive2,
        //         AnalogOperation.DriveTrainPositionDrive3,
        //         AnalogOperation.DriveTrainPositionDrive4,
        //         AnalogOperation.DriveTrainPositionSteer1,
        //         AnalogOperation.DriveTrainPositionSteer2,
        //         AnalogOperation.DriveTrainPositionSteer3,
        //         AnalogOperation.DriveTrainPositionSteer4,
        //         DigitalOperation.DriveTrainSteerMode,
        //         DigitalOperation.DriveTrainMaintainPositionMode,
        //         DigitalOperation.DriveTrainPathMode,
        //         DigitalOperation.DriveTrainReset,
        //         DigitalOperation.DriveTrainEnableFieldOrientation,
        //         DigitalOperation.DriveTrainDisableFieldOrientation,
        //         DigitalOperation.DriveTrainUseRobotOrientation,
        //     }),
        // new MacroOperationDescription(
        //     MacroOperation.PIDHeavyBrake,
        //     UserInputDevice.Driver,
        //     UserInputDeviceButton.XBONE_LEFT_STICK_BUTTON, // DPAD-right
        //     EnumSet.of(Shift.DriverDebug),
        //     EnumSet.of(Shift.DriverDebug),
        //     ButtonType.Simple,
        //     () -> new PIDBrakeTask(true),
        //     new IOperation[]
        //     {
        //         AnalogOperation.DriveTrainMoveForward,
        //         AnalogOperation.DriveTrainMoveRight,
        //         AnalogOperation.DriveTrainTurnAngleGoal,
        //         AnalogOperation.DriveTrainSpinLeft,
        //         AnalogOperation.DriveTrainSpinRight,
        //         AnalogOperation.DriveTrainRotationA,
        //         AnalogOperation.DriveTrainRotationB,
        //         AnalogOperation.DriveTrainPathXGoal,
        //         AnalogOperation.DriveTrainPathYGoal,
        //         AnalogOperation.DriveTrainPathXVelocityGoal,
        //         AnalogOperation.DriveTrainPathYVelocityGoal,
        //         AnalogOperation.DriveTrainPathAngleGoal,
        //         AnalogOperation.DriveTrainPathAngleVelocityGoal,
        //         AnalogOperation.DriveTrainPositionDrive1,
        //         AnalogOperation.DriveTrainPositionDrive2,
        //         AnalogOperation.DriveTrainPositionDrive3,
        //         AnalogOperation.DriveTrainPositionDrive4,
        //         AnalogOperation.DriveTrainPositionSteer1,
        //         AnalogOperation.DriveTrainPositionSteer2,
        //         AnalogOperation.DriveTrainPositionSteer3,
        //         AnalogOperation.DriveTrainPositionSteer4,
        //         DigitalOperation.DriveTrainSteerMode,
        //         DigitalOperation.DriveTrainMaintainPositionMode,
        //         DigitalOperation.DriveTrainPathMode,
        //         DigitalOperation.DriveTrainReset,
        //         DigitalOperation.DriveTrainEnableFieldOrientation,
        //         DigitalOperation.DriveTrainDisableFieldOrientation,
        //         DigitalOperation.DriveTrainUseRobotOrientation,
        //     }),

        // new MacroOperationDescription(
        //     MacroOperation.FaceForward,
        //     UserInputDevice.Driver,
        //     0, // DPAD-up
        //     EnumSet.of(Shift.DriverDebug),
        //     EnumSet.noneOf(Shift.class),
        //     ButtonType.Toggle,
        //     () -> new OrientationTask(0),
        //     new IOperation[]
        //     {
        //         AnalogOperation.DriveTrainTurnAngleGoal,
        //         AnalogOperation.DriveTrainSpinLeft,
        //         AnalogOperation.DriveTrainSpinRight,
        //     }),

        // new MacroOperationDescription(
        //     MacroOperation.FaceSomething,
        //     UserInputDevice.Driver,
        //     UserInputDeviceButton.XBONE_B_BUTTON, // DPAD-up
        //     EnumSet.of(Shift.DriverDebug),
        //     EnumSet.of(Shift.DriverDebug),
        //     ButtonType.Toggle,
        //     () -> new FieldOrientationTask(DesiredOrientation.Something),
        //     new IOperation[]
        //     {
        //         AnalogOperation.DriveTrainTurnAngleGoal,
        //         AnalogOperation.DriveTrainSpinLeft,
        //         AnalogOperation.DriveTrainSpinRight,
        //     }),

        // new MacroOperationDescription(
        //     MacroOperation.FaceLeft,
        //     UserInputDevice.Driver,
        //     270, // DPAD-left
        //     EnumSet.of(Shift.DriverDebug),
        //     EnumSet.noneOf(Shift.class),
        //     ButtonType.Toggle,
        //     () -> new OrientationTask(90),
        //     new IOperation[]
        //     {
        //         AnalogOperation.DriveTrainTurnAngleGoal,
        //         AnalogOperation.DriveTrainSpinLeft,
        //         AnalogOperation.DriveTrainSpinRight,
        //     }),

        // new MacroOperationDescription(
        //     MacroOperation.FaceRight,
        //     UserInputDevice.Driver,
        //     90, // DPAD-right
        //     EnumSet.of(Shift.DriverDebug),
        //     EnumSet.noneOf(Shift.class),
        //     ButtonType.Toggle,
        //     () -> new OrientationTask(-90),
        //     new IOperation[]
        //     {
        //         AnalogOperation.DriveTrainTurnAngleGoal,
        //         AnalogOperation.DriveTrainSpinLeft,
        //         AnalogOperation.DriveTrainSpinRight,
        //     }),

        // new MacroOperationDescription(
        //     MacroOperation.FaceBackward,
        //     UserInputDevice.Driver,
        //     180, // DPAD-down
        //     EnumSet.of(Shift.DriverDebug),
        //     EnumSet.noneOf(Shift.class),
        //     ButtonType.Toggle,
        //     () -> new OrientationTask(180),
        //     new IOperation[]
        //     {
        //         AnalogOperation.DriveTrainTurnAngleGoal,
        //         AnalogOperation.DriveTrainSpinLeft,
        //         AnalogOperation.DriveTrainSpinRight,
        //     }),

        // Wrist Operations
        new MacroOperationDescription(
            MacroOperation.LowCubeDrop,
            UserInputDevice.Driver,
            180,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> new WristPositionTask(TuningConstants.LOW_CUBE_DROP_POSITION),
            new IOperation[]
            {
                AnalogOperation.WristSetAngle
            }),
        new MacroOperationDescription(
            MacroOperation.MidCubeDrop,
            UserInputDevice.Driver,
            270,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> new WristPositionTask(TuningConstants.MID_CUBE_DROP_POSITION),
            new IOperation[]
            {
                AnalogOperation.WristSetAngle
            }),
        new MacroOperationDescription(
            MacroOperation.HighCubeDrop,
            UserInputDevice.Driver,
            0,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> new WristPositionTask(TuningConstants.HIGH_CUBE_DROP_POSITION),
            new IOperation[]
            {
                AnalogOperation.WristSetAngle
            }),
        new MacroOperationDescription(
            MacroOperation.WristStowed,
            UserInputDevice.Driver,
            90,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> new WristPositionTask(TuningConstants.STOWED_POSITION),
            new IOperation[]
            {
                AnalogOperation.WristSetAngle
            }),
        new MacroOperationDescription(
            MacroOperation.GroundPickup,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_RIGHT_BUTTON,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> new WristPositionTask(TuningConstants.GROUND_CUBE_PICKUP),
            new IOperation[]
            {
                AnalogOperation.WristSetAngle
            }),
        new MacroOperationDescription(
            MacroOperation.SubstationPickup,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_RIGHT_BUTTON,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.of(Shift.DriverDebug),
            ButtonType.Toggle,
            () -> new WristPositionTask(TuningConstants.SUBSTATION_CUBE_PICKUP),
            new IOperation[]
            {
                AnalogOperation.WristSetAngle
            }),

        // new MacroOperationDescription(
        //     MacroOperation.FollowPathTest1,
        //     UserInputDevice.Test1,
        //     0,
        //     EnumSet.of(Shift.Test1Debug),
        //     EnumSet.noneOf(Shift.class),
        //     ButtonType.Toggle,
        //     () -> SequentialTask.Sequence(
        //         new FollowPathTask("goLeft32inForward18in", Type.RobotRelativeFromCurrentPose)
        //     ),
        //     new IOperation[]
        //     {
        //         DigitalOperation.PositionResetFieldOrientation,
        //         DigitalOperation.PositionResetRobotLevel,
        //         AnalogOperation.PositionStartingAngle,
        //         DigitalOperation.DriveTrainResetXYPosition,
        //         DigitalOperation.DriveTrainEnableMaintainDirectionMode,
        //         AnalogOperation.DriveTrainStartingXPosition,
        //         AnalogOperation.DriveTrainStartingYPosition,
        //         AnalogOperation.DriveTrainMoveForward,
        //         AnalogOperation.DriveTrainMoveRight,
        //         AnalogOperation.DriveTrainTurnAngleGoal,
        //         AnalogOperation.DriveTrainSpinLeft,
        //         AnalogOperation.DriveTrainSpinRight,
        //         AnalogOperation.DriveTrainRotationA,
        //         AnalogOperation.DriveTrainRotationB,
        //         AnalogOperation.DriveTrainPathXGoal,
        //         AnalogOperation.DriveTrainPathYGoal,
        //         AnalogOperation.DriveTrainPathXVelocityGoal,
        //         AnalogOperation.DriveTrainPathYVelocityGoal,
        //         AnalogOperation.DriveTrainPathAngleVelocityGoal,
        //         AnalogOperation.DriveTrainPositionDrive1,
        //         AnalogOperation.DriveTrainPositionDrive2,
        //         AnalogOperation.DriveTrainPositionDrive3,
        //         AnalogOperation.DriveTrainPositionDrive4,
        //         AnalogOperation.DriveTrainPositionSteer1,
        //         AnalogOperation.DriveTrainPositionSteer2,
        //         AnalogOperation.DriveTrainPositionSteer3,
        //         AnalogOperation.DriveTrainPositionSteer4,
        //         DigitalOperation.DriveTrainSteerMode,
        //         DigitalOperation.DriveTrainMaintainPositionMode,
        //         DigitalOperation.DriveTrainPathMode,
        //         DigitalOperation.DriveTrainReset,
        //         DigitalOperation.DriveTrainEnableFieldOrientation,
        //         DigitalOperation.DriveTrainDisableFieldOrientation,
        //         DigitalOperation.DriveTrainUseRobotOrientation,
        //         DigitalOperation.VisionEnableStream,
        //         DigitalOperation.VisionFindSpecificAprilTag,
        //         DigitalOperation.VisionFindAnyAprilTag,
        //         DigitalOperation.VisionFindAbsolutePosition,
        //         DigitalOperation.VisionForceDisable,
        //     }),


        new MacroOperationDescription(
            MacroOperation.Test2024_2,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_X_BUTTON,
            EnumSet.of(Shift.Test1Debug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
             () -> SequentialTask.Sequence(
                
                 ConcurrentTask.AllTasks(),
                    new ResetLevelTask(),
                    new PositionStartingTask(
                        
                        250.5+halfFramePreremetere,
                        306-halfFramePreremetere,
                        180.0,
                        true,
                        true),
      
                 new FollowPathTask("P3toP8Red", Type.Absolute)),


            


                 
                 
        new IOperation[]
        {
            DigitalOperation.PositionResetFieldOrientation,
            DigitalOperation.PositionResetRobotLevel,
            AnalogOperation.PositionStartingAngle,
            DigitalOperation.DriveTrainResetXYPosition,
            DigitalOperation.DriveTrainEnableMaintainDirectionMode,
            AnalogOperation.DriveTrainStartingXPosition,
            AnalogOperation.DriveTrainStartingYPosition,
            AnalogOperation.DriveTrainMoveForward,
            AnalogOperation.DriveTrainMoveRight,
            AnalogOperation.DriveTrainTurnAngleGoal,
            AnalogOperation.DriveTrainSpinLeft,
            AnalogOperation.DriveTrainSpinRight,
            AnalogOperation.DriveTrainRotationA,
            AnalogOperation.DriveTrainRotationB,
            AnalogOperation.DriveTrainPathXGoal,
            AnalogOperation.DriveTrainPathYGoal,
            AnalogOperation.DriveTrainPathXVelocityGoal,
            AnalogOperation.DriveTrainPathYVelocityGoal,
            AnalogOperation.DriveTrainPathAngleVelocityGoal,
            AnalogOperation.DriveTrainPositionDrive1,
            AnalogOperation.DriveTrainPositionDrive2,
            AnalogOperation.DriveTrainPositionDrive3,
            AnalogOperation.DriveTrainPositionDrive4,
            AnalogOperation.DriveTrainPositionSteer1,
            AnalogOperation.DriveTrainPositionSteer2,
            AnalogOperation.DriveTrainPositionSteer3,
            AnalogOperation.DriveTrainPositionSteer4,
            DigitalOperation.DriveTrainSteerMode,
            DigitalOperation.DriveTrainMaintainPositionMode,
            DigitalOperation.DriveTrainPathMode,
            DigitalOperation.DriveTrainReset,
            DigitalOperation.DriveTrainEnableFieldOrientation,
            DigitalOperation.DriveTrainDisableFieldOrientation,
            DigitalOperation.DriveTrainUseRobotOrientation,
            DigitalOperation.VisionEnableStream,
            DigitalOperation.VisionFindSpecificAprilTag,
            DigitalOperation.VisionFindAnyAprilTag,
            DigitalOperation.VisionFindAbsolutePosition,
            DigitalOperation.VisionForceDisable,
        }),


        



        /* new MacroOperationDescription(
            MacroOperation.Test2024_2,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_X_BUTTON,
            EnumSet.of(Shift.Test1Debug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
             () -> SequentialTask.Sequence(
                
                 ConcurrentTask.AllTasks(),
                    new ResetLevelTask(),
                    new PositionStartingTask(
                        //Change tour x - axis value based on is red
                        //P3 settings
                        //250.5 + 11.25,
                        //306 - 11.25,
                        100.0,
                        100.0,
                        0.0,
                        true,
                        true),
      
                 new FollowPathTask("JamieGoBack", Type.Absolute)),
                 
                 
        new IOperation[]
        {
            DigitalOperation.PositionResetFieldOrientation,
            DigitalOperation.PositionResetRobotLevel,
            AnalogOperation.PositionStartingAngle,
            DigitalOperation.DriveTrainResetXYPosition,
            DigitalOperation.DriveTrainEnableMaintainDirectionMode,
            AnalogOperation.DriveTrainStartingXPosition,
            AnalogOperation.DriveTrainStartingYPosition,
            AnalogOperation.DriveTrainMoveForward,
            AnalogOperation.DriveTrainMoveRight,
            AnalogOperation.DriveTrainTurnAngleGoal,
            AnalogOperation.DriveTrainSpinLeft,
            AnalogOperation.DriveTrainSpinRight,
            AnalogOperation.DriveTrainRotationA,
            AnalogOperation.DriveTrainRotationB,
            AnalogOperation.DriveTrainPathXGoal,
            AnalogOperation.DriveTrainPathYGoal,
            AnalogOperation.DriveTrainPathXVelocityGoal,
            AnalogOperation.DriveTrainPathYVelocityGoal,
            AnalogOperation.DriveTrainPathAngleVelocityGoal,
            AnalogOperation.DriveTrainPositionDrive1,
            AnalogOperation.DriveTrainPositionDrive2,
            AnalogOperation.DriveTrainPositionDrive3,
            AnalogOperation.DriveTrainPositionDrive4,
            AnalogOperation.DriveTrainPositionSteer1,
            AnalogOperation.DriveTrainPositionSteer2,
            AnalogOperation.DriveTrainPositionSteer3,
            AnalogOperation.DriveTrainPositionSteer4,
            DigitalOperation.DriveTrainSteerMode,
            DigitalOperation.DriveTrainMaintainPositionMode,
            DigitalOperation.DriveTrainPathMode,
            DigitalOperation.DriveTrainReset,
            DigitalOperation.DriveTrainEnableFieldOrientation,
            DigitalOperation.DriveTrainDisableFieldOrientation,
            DigitalOperation.DriveTrainUseRobotOrientation,
            DigitalOperation.VisionEnableStream,
            DigitalOperation.VisionFindSpecificAprilTag,
            DigitalOperation.VisionFindAnyAprilTag,
            DigitalOperation.VisionFindAbsolutePosition,
            DigitalOperation.VisionForceDisable,
        }), */


        new MacroOperationDescription(
        MacroOperation.Test2024_8,
        UserInputDevice.Test1,
        UserInputDeviceButton.XBONE_B_BUTTON,
        EnumSet.of(Shift.Test1Debug),
        EnumSet.noneOf(Shift.class),
        ButtonType.Toggle,
            () -> SequentialTask.Sequence(
            
                ConcurrentTask.AllTasks(),
                new ResetLevelTask(),
                new PositionStartingTask(

                    //this is P4
                    //(288 - halfFramePreremetere), (239 - halfFramePreremetere),


                    //this is P3
                    //250.5 + halfFramePreremetere, 306 - halfFramePreremetere,

                    //this is P1
                    326.5 - halfFramePreremetere, 64 + halfFramePreremetere,

                    180.0,
                    true,
                    true),
                //new FollowPathTask( "P3toP8Red", Type.Absolute)),
                new FollowPathTask( "P1toP12Red", Type.Absolute)),
                new IOperation[]
                {
                    DigitalOperation.PositionResetFieldOrientation,
                    DigitalOperation.PositionResetRobotLevel,
                    AnalogOperation.PositionStartingAngle,
                    DigitalOperation.DriveTrainResetXYPosition,
                    DigitalOperation.DriveTrainEnableMaintainDirectionMode,
                    AnalogOperation.DriveTrainStartingXPosition,
                    AnalogOperation.DriveTrainStartingYPosition,
                    AnalogOperation.DriveTrainMoveForward,
                    AnalogOperation.DriveTrainMoveRight,
                    AnalogOperation.DriveTrainTurnAngleGoal,
                    AnalogOperation.DriveTrainSpinLeft,
                    AnalogOperation.DriveTrainSpinRight,
                    AnalogOperation.DriveTrainRotationA,
                    AnalogOperation.DriveTrainRotationB,
                    AnalogOperation.DriveTrainPathXGoal,
                    AnalogOperation.DriveTrainPathYGoal,
                    AnalogOperation.DriveTrainPathXVelocityGoal,
                    AnalogOperation.DriveTrainPathYVelocityGoal,
                    AnalogOperation.DriveTrainPathAngleVelocityGoal,
                    AnalogOperation.DriveTrainPositionDrive1,
                    AnalogOperation.DriveTrainPositionDrive2,
                    AnalogOperation.DriveTrainPositionDrive3,
                    AnalogOperation.DriveTrainPositionDrive4,
                    AnalogOperation.DriveTrainPositionSteer1,
                    AnalogOperation.DriveTrainPositionSteer2,
                    AnalogOperation.DriveTrainPositionSteer3,
                    AnalogOperation.DriveTrainPositionSteer4,
                    DigitalOperation.DriveTrainSteerMode,
                    DigitalOperation.DriveTrainMaintainPositionMode,
                    DigitalOperation.DriveTrainPathMode,
                    DigitalOperation.DriveTrainReset,
                    DigitalOperation.DriveTrainEnableFieldOrientation,
                    DigitalOperation.DriveTrainDisableFieldOrientation,
                    DigitalOperation.DriveTrainUseRobotOrientation,
                    DigitalOperation.VisionEnableStream,
                    DigitalOperation.VisionFindSpecificAprilTag,
                    DigitalOperation.VisionFindAnyAprilTag,
                    DigitalOperation.VisionFindAbsolutePosition,
                    DigitalOperation.VisionForceDisable,
                }),

        new MacroOperationDescription(
        MacroOperation.Test2024_3,
        UserInputDevice.Test1,
        UserInputDeviceButton.XBONE_SELECT_BUTTON,
        EnumSet.of(Shift.Test1Debug),
        EnumSet.noneOf(Shift.class),
        ButtonType.Toggle,
            () -> SequentialTask.Sequence(
            
                ConcurrentTask.AllTasks(),
                new ResetLevelTask(),
                new PositionStartingTask(

                    //this is P4
                    (288 - halfFramePreremetere), (239 - halfFramePreremetere),


                    //this is P3
                    //250.5 + halfFramePreremetere, 306 - halfFramePreremetere,

                    //this is P1
                    //250 + halfFramePreremetere, 23 + halfFramePreremetere,

                    180.0,
                    true,
                    true),
    
                new FollowPathTask("SevenNoteAutoRed", Type.Absolute)),
            
                new IOperation[]
                {
                    DigitalOperation.PositionResetFieldOrientation,
                    DigitalOperation.PositionResetRobotLevel,
                    AnalogOperation.PositionStartingAngle,
                    DigitalOperation.DriveTrainResetXYPosition,
                    DigitalOperation.DriveTrainEnableMaintainDirectionMode,
                    AnalogOperation.DriveTrainStartingXPosition,
                    AnalogOperation.DriveTrainStartingYPosition,
                    AnalogOperation.DriveTrainMoveForward,
                    AnalogOperation.DriveTrainMoveRight,
                    AnalogOperation.DriveTrainTurnAngleGoal,
                    AnalogOperation.DriveTrainSpinLeft,
                    AnalogOperation.DriveTrainSpinRight,
                    AnalogOperation.DriveTrainRotationA,
                    AnalogOperation.DriveTrainRotationB,
                    AnalogOperation.DriveTrainPathXGoal,
                    AnalogOperation.DriveTrainPathYGoal,
                    AnalogOperation.DriveTrainPathXVelocityGoal,
                    AnalogOperation.DriveTrainPathYVelocityGoal,
                    AnalogOperation.DriveTrainPathAngleVelocityGoal,
                    AnalogOperation.DriveTrainPositionDrive1,
                    AnalogOperation.DriveTrainPositionDrive2,
                    AnalogOperation.DriveTrainPositionDrive3,
                    AnalogOperation.DriveTrainPositionDrive4,
                    AnalogOperation.DriveTrainPositionSteer1,
                    AnalogOperation.DriveTrainPositionSteer2,
                    AnalogOperation.DriveTrainPositionSteer3,
                    AnalogOperation.DriveTrainPositionSteer4,
                    DigitalOperation.DriveTrainSteerMode,
                    DigitalOperation.DriveTrainMaintainPositionMode,
                    DigitalOperation.DriveTrainPathMode,
                    DigitalOperation.DriveTrainReset,
                    DigitalOperation.DriveTrainEnableFieldOrientation,
                    DigitalOperation.DriveTrainDisableFieldOrientation,
                    DigitalOperation.DriveTrainUseRobotOrientation,
                    DigitalOperation.VisionEnableStream,
                    DigitalOperation.VisionFindSpecificAprilTag,
                    DigitalOperation.VisionFindAnyAprilTag,
                    DigitalOperation.VisionFindAbsolutePosition,
                    DigitalOperation.VisionForceDisable,
                }),


        new MacroOperationDescription(
            MacroOperation.Test2024_4,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_START_BUTTON,
            EnumSet.of(Shift.Test1Debug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
                () -> SequentialTask.Sequence(
                
                    ConcurrentTask.AllTasks(),
                    new ResetLevelTask(),
                    new PositionStartingTask(
                        //Change tour x - axis value based on is red
                        //P3 settings
                        //250.5 + 11.25,
                        //306 - 11.25,
                        100.0,
                        100.0,
                        0.0,
                        true,
                        true),
        
                    new FollowPathTask("JamieGoRight", Type.Absolute)),
        
        new IOperation[]
        {
            DigitalOperation.PositionResetFieldOrientation,
            DigitalOperation.PositionResetRobotLevel,
            AnalogOperation.PositionStartingAngle,
            DigitalOperation.DriveTrainResetXYPosition,
            DigitalOperation.DriveTrainEnableMaintainDirectionMode,
            AnalogOperation.DriveTrainStartingXPosition,
            AnalogOperation.DriveTrainStartingYPosition,
            AnalogOperation.DriveTrainMoveForward,
            AnalogOperation.DriveTrainMoveRight,
            AnalogOperation.DriveTrainTurnAngleGoal,
            AnalogOperation.DriveTrainSpinLeft,
            AnalogOperation.DriveTrainSpinRight,
            AnalogOperation.DriveTrainRotationA,
            AnalogOperation.DriveTrainRotationB,
            AnalogOperation.DriveTrainPathXGoal,
            AnalogOperation.DriveTrainPathYGoal,
            AnalogOperation.DriveTrainPathXVelocityGoal,
            AnalogOperation.DriveTrainPathYVelocityGoal,
            AnalogOperation.DriveTrainPathAngleVelocityGoal,
            AnalogOperation.DriveTrainPositionDrive1,
            AnalogOperation.DriveTrainPositionDrive2,
            AnalogOperation.DriveTrainPositionDrive3,
            AnalogOperation.DriveTrainPositionDrive4,
            AnalogOperation.DriveTrainPositionSteer1,
            AnalogOperation.DriveTrainPositionSteer2,
            AnalogOperation.DriveTrainPositionSteer3,
            AnalogOperation.DriveTrainPositionSteer4,
            DigitalOperation.DriveTrainSteerMode,
            DigitalOperation.DriveTrainMaintainPositionMode,
            DigitalOperation.DriveTrainPathMode,
            DigitalOperation.DriveTrainReset,
            DigitalOperation.DriveTrainEnableFieldOrientation,
            DigitalOperation.DriveTrainDisableFieldOrientation,
            DigitalOperation.DriveTrainUseRobotOrientation,
            DigitalOperation.VisionEnableStream,
            DigitalOperation.VisionFindSpecificAprilTag,
            DigitalOperation.VisionFindAnyAprilTag,
            DigitalOperation.VisionFindAbsolutePosition,
            DigitalOperation.VisionForceDisable,
        }),


        // Full auton test
        new MacroOperationDescription(
            MacroOperation.FollowPathTest2,
            UserInputDevice.Test1,  
            180,
            EnumSet.noneOf(Shift.class),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> new FollowPathTask("goLeft30in", Type.RobotRelativeFromCurrentPose),
            new IOperation[]
            {
                DigitalOperation.PositionResetFieldOrientation,
                DigitalOperation.PositionResetRobotLevel,
                AnalogOperation.PositionStartingAngle,
                DigitalOperation.DriveTrainResetXYPosition,
                AnalogOperation.DriveTrainStartingXPosition,
                AnalogOperation.DriveTrainStartingYPosition,
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
                DigitalOperation.VisionEnableStream,
                DigitalOperation.VisionFindSpecificAprilTag,
                DigitalOperation.VisionFindAnyAprilTag,
                DigitalOperation.VisionFindAbsolutePosition,
                DigitalOperation.VisionForceDisable,
            }),

        new MacroOperationDescription(
            MacroOperation.FollowPathTest3,
            UserInputDevice.Test1,
            270,
            EnumSet.noneOf(Shift.class),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> new FollowPathTask("goLeft22in", Type.RobotRelativeFromCurrentPose),
            new IOperation[]
            {
                DigitalOperation.PositionResetFieldOrientation,
                DigitalOperation.PositionResetRobotLevel,
                AnalogOperation.PositionStartingAngle,
                DigitalOperation.DriveTrainResetXYPosition,
                AnalogOperation.DriveTrainStartingXPosition,
                AnalogOperation.DriveTrainStartingYPosition,
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
                DigitalOperation.VisionEnableStream,
                DigitalOperation.VisionFindSpecificAprilTag,
                DigitalOperation.VisionFindAnyAprilTag,
                DigitalOperation.VisionFindAbsolutePosition,
                DigitalOperation.VisionForceDisable,
            }),

        new MacroOperationDescription(
            MacroOperation.FollowPathTest4,
            UserInputDevice.Test2,
            UserInputDeviceButton.XBONE_RIGHT_BUTTON,
            EnumSet.noneOf(Shift.class),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> new FollowPathTask("goLeft22in", Type.RobotRelativeFromCurrentPose),
            new IOperation[]
            {
                DigitalOperation.PositionResetFieldOrientation,
                DigitalOperation.PositionResetRobotLevel,
                AnalogOperation.PositionStartingAngle,
                DigitalOperation.DriveTrainResetXYPosition,
                AnalogOperation.DriveTrainStartingXPosition,
                AnalogOperation.DriveTrainStartingYPosition,
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
                DigitalOperation.VisionEnableStream,
                DigitalOperation.VisionFindSpecificAprilTag,
                DigitalOperation.VisionFindAnyAprilTag,
                DigitalOperation.VisionFindAbsolutePosition,
                DigitalOperation.VisionForceDisable,
            }),
    };

    @Override
    public ShiftDescription[] getShiftSchema()
    {
        return ButtonMap.ShiftButtonSchema;
    }

    @Override
    public AnalogOperationDescription[] getAnalogOperationSchema()
    {
        return ButtonMap.AnalogOperationSchema;
    }

    @Override
    public DigitalOperationDescription[] getDigitalOperationSchema()
    {
        return ButtonMap.DigitalOperationSchema;
    }

    @Override
    public MacroOperationDescription[] getMacroOperationSchema()
    {
        return ButtonMap.MacroSchema;
    }
}
