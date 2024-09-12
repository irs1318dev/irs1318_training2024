package frc.robot.mechanisms;

import frc.lib.driver.IDriver;
import frc.lib.mechanisms.IMechanism;
import frc.lib.robotprovider.*;
import frc.robot.TuningConstants;
import frc.robot.driver.DigitalOperation;
import frc.lib.driver.descriptions.UserInputDevice;
import frc.lib.helpers.Helpers;
import frc.robot.mechanisms.PowerManager.CurrentLimiting;

import com.google.inject.Inject;
import com.google.inject.Singleton;

/**
 * Driver feedback manager
 *
 * This class manages things like controller rumbler and indicator lights on the robot.
 *
 */
@Singleton
public class DriverFeedbackManager implements IMechanism
{
    private final IDriverStation ds;
    private final IDriver driver;
    private final ITimer timer;

    private final PowerManager powerMan;

    @Inject
    public DriverFeedbackManager(
        IDriver driver,
        IRobotProvider provider,
        ITimer timer,
        PowerManager powerMan)
    {
        this.driver = driver;
        this.timer = timer;

        this.ds = provider.getDriverStation();
        this.powerMan = powerMan;
    }

    @Override
    public void readSensors()
    {
    }

    @Override
    public void update(RobotMode mode)
    {
        boolean isCurrentLimiting = this.powerMan.getCurrentLimitingValue() != CurrentLimiting.Normal;

        boolean driverShouldRumble = false;
        boolean codriverShouldRumble = false;
        if (mode == RobotMode.Teleop)
        {
            if (this.ds.isFMSMode() &&
                Helpers.WithinRange(this.ds.getMatchTime(), TuningConstants.ENDGAME_RUMBLE - 3.0, TuningConstants.ENDGAME_RUMBLE))
            {
                this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Right, 0.25);
                driverShouldRumble = true;
            }
        }

        if (this.driver.getDigital(DigitalOperation.ForceLightDriverRumble))
        {
            this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Left, 0.25);
            this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Right, 0.25);
            driverShouldRumble = true;

            this.driver.setRumble(UserInputDevice.Codriver, JoystickRumbleType.Left, 0.0);
            this.driver.setRumble(UserInputDevice.Codriver, JoystickRumbleType.Right, 0.0);
            codriverShouldRumble = true;
        }

        if (!driverShouldRumble)
        {
            this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Left, 0.0);
            this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Right, 0.0);
        }

        if (!codriverShouldRumble)
        {
            this.driver.setRumble(UserInputDevice.Codriver, JoystickRumbleType.Left, 0.0);
            this.driver.setRumble(UserInputDevice.Codriver, JoystickRumbleType.Right, 0.0);
        }
    }

    @Override
    public void stop()
    {
        this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Left, 0.0);
        this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Right, 0.0);
    }
}
