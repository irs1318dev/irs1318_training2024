package frc.lib.robotprovider;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class JoystickWrapper implements IJoystick
{
    private final Joystick wrappedObject;

    public JoystickWrapper(int port)
    {
        this.wrappedObject = new Joystick(port);
    }

    public boolean isConnected()
    {
        return this.wrappedObject.isConnected();
    }

    public double getAxis(int relevantAxis)
    {
        return this.wrappedObject.getRawAxis(relevantAxis);
    }

    public int getPOV()
    {
        return this.wrappedObject.getPOV();
    }

    public boolean getRawButton(int value)
    {
        return this.wrappedObject.getRawButton(value);
    }

    public void setRumble(JoystickRumbleType type, double value)
    {
        RumbleType rumbleType;
        if (type == JoystickRumbleType.Left)
        {
            rumbleType = RumbleType.kLeftRumble;
        }
        else // if (type == JoystickRumbleType.Right)
        {
            rumbleType = RumbleType.kRightRumble;
        }

        this.wrappedObject.setRumble(rumbleType, value);
    }
}
