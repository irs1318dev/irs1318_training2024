package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.google.inject.Injector;

import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.CoreRobot;
import frc.lib.robotprovider.IFile;
import frc.lib.robotprovider.IRobotProvider;

/**
 * Robot wraps CoreRobot to allow for the basic autonomous/teleop and switching logic to be shared between
 * Robot and Fauxbot (and whatever may come after).
 * 
 * The FRC Robot VM is designed to call the functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package, you must also update the Main class that
 * references it.
 */
public class AKRobot extends LoggedRobot
{
    private final CoreRobot<RobotModule> robot;

    public AKRobot()
    {
        this.robot = new CoreRobot<RobotModule>(new RobotModule());
    }

    /**
     * Robot-wide initialization code should go here.
     * This default Robot-wide initialization code will be called when 
     * the robot is first powered on.  It will be called exactly 1 time.
     */
    @Override
    public void robotInit()
    {
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_GROUP);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

        switch (BuildConstants.DIRTY)
        {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;

            case 1:
                Logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;

            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        if (RobotBase.isReal())
        {
            // Running on a real robot, attempt to log to a USB stick ("/U/logs") if one is plugged in
            Injector injector = this.robot.getInjector();
            IFile rootDirectory = injector.getInstance(IFile.class);
            rootDirectory.open("/U/");
            if (rootDirectory.exists())
            {
                Logger.addDataReceiver(new WPILOGWriter());
            }

            Logger.addDataReceiver(new NT4Publisher());
        }
        else if (RobotBase.isSimulation())
        {
            Logger.addDataReceiver(new NT4Publisher());
        }
        else
        {
            // Replaying a log, set up replay source
            setUseTiming(false); // Run as fast as possible
            String logPath = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        }

        if (TuningConstants.RETREIVE_PDH_FIRST)
        {
            // retrieve the power distribution hub before we start the logger to avoid double-initialization
            this.robot.getInjector()
                .getInstance(IRobotProvider.class)
                .getPowerDistribution(ElectronicsConstants.POWER_DISTRIBUTION_CAN_ID, ElectronicsConstants.POWER_DISTRIBUTION_TYPE);
        }

        // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
        Logger.start();

        this.robot.robotInit();
    }

    /**
     * Initialization code for disabled mode should go here.
     * This code will be called each time the robot enters disabled mode.
     */
    @Override
    public void disabledInit()
    {
        this.robot.disabledInit();
    }

    /**
     * Initialization code for autonomous mode should go here.
     * This code will be called each time the robot enters autonomous mode.
     */
    @Override
    public void autonomousInit()
    {
        this.robot.autonomousInit();
    }

    /**
     * Initialization code for teleop mode should go here.
     * This code will be called each time the robot enters teleop mode.
     */
    @Override
    public void teleopInit()
    {
        this.robot.teleopInit();
    }

    /**
     * Initialization code for test mode should go here.
     * This code will be called each time the robot enters test mode.
     */
    @Override
    public void testInit()
    {
        this.robot.testInit();
    }

    /**
     * Initialization code for simulation mode should go here.
     * This code will be called each time the robot enters simulation mode.
     */
    @Override
    public void simulationInit()
    {
        this.robot.simulationInit();
    }

    /**
     * Periodic code for disabled mode should go here.
     * This code will be called periodically at a regular rate while the robot is in disabled mode.
     */
    @Override
    public void disabledPeriodic()
    {
        this.robot.disabledPeriodic();
    }

    /**
     * Periodic code for autonomous mode should go here.
     * This code will be called periodically at a regular rate while the robot is in autonomous mode.
     */
    @Override
    public void autonomousPeriodic()
    {
        this.robot.autonomousPeriodic();
    }

    /**
     * Periodic code for teleop mode should go here.
     * This code will be called periodically at a regular rate while the robot is in teleop mode.
     */
    @Override
    public void teleopPeriodic()
    {
        this.robot.teleopPeriodic();
    }

    /**
     * Periodic code for test mode should go here.
     * This code will be called periodically at a regular rate while the robot is in test mode.
     */
    @Override
    public void testPeriodic()
    {
        this.robot.testPeriodic();
    }
  
    /**
     * Periodic code for simulation mode should go here.
     * This code will be called periodically at a regular rate while the robot is in simulation mode.
     */
    @Override
    public void simulationPeriodic()
    {
        this.robot.simulationPeriodic();
    }
}
