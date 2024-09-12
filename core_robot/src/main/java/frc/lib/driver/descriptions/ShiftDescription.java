package frc.lib.driver.descriptions;

import frc.robot.TuningConstants;
import frc.lib.driver.AnalogAxis;
import frc.lib.driver.UserInputDeviceButton;
import frc.lib.helpers.ExceptionHelpers;
import frc.lib.robotprovider.IJoystick;
import frc.robot.driver.Shift;

public class ShiftDescription
{
    private final Shift shift;
    private final UserInputDevice userInputDevice;
    private final UserInputDeviceButton userInputDeviceButton;
    private final int userInputDevicePovValue;
    private final AnalogAxis userInputDeviceAxis;
    private final double userInputDeviceAxisRangeMin;
    private final double userInputDeviceAxisRangeMax;

    public ShiftDescription(Shift shift, UserInputDevice userInputDevice, UserInputDeviceButton userInputDeviceButton)
    {
        this(shift, userInputDevice, userInputDeviceButton, -1, AnalogAxis.NONE, 0.0, 0.0);
    }

    public ShiftDescription(Shift shift, UserInputDevice userInputDevice, int povValue)
    {
        this(shift, userInputDevice, UserInputDeviceButton.POV, povValue, AnalogAxis.NONE, 0.0, 0.0);
    }

    public ShiftDescription(
        Shift shift,
        UserInputDevice userInputDevice,
        AnalogAxis analogAxis,
        double axisRangeMinValue,
        double axisRangeMaxValue)
    {
        this(shift, userInputDevice, UserInputDeviceButton.ANALOG_AXIS_RANGE, -1, analogAxis, axisRangeMinValue, axisRangeMaxValue);
    }

    private ShiftDescription(
        Shift shift,
        UserInputDevice userInputDevice,
        UserInputDeviceButton userInputDeviceButton,
        int povValue,
        AnalogAxis analogAxis,
        double axisRangeMinValue,
        double axisRangeMaxValue)
    {
        this.shift = shift;
        this.userInputDevice = userInputDevice;
        this.userInputDeviceButton = userInputDeviceButton;
        this.userInputDevicePovValue = povValue;
        this.userInputDeviceAxis = analogAxis;
        this.userInputDeviceAxisRangeMin = axisRangeMinValue;
        this.userInputDeviceAxisRangeMax = axisRangeMaxValue;
    }

    public Shift getShift()
    {
        return this.shift;
    }

    public UserInputDevice getUserInputDevice()
    {
        return this.userInputDevice;
    }

    public UserInputDeviceButton getUserInputDeviceButton()
    {
        return this.userInputDeviceButton;
    }

    public int getUserInputDevicePovValue()
    {
        return this.userInputDevicePovValue;
    }

    public boolean checkInput(IJoystick[] joysticks)
    {
        UserInputDevice userInputDevice = this.getUserInputDevice();
        IJoystick relevantJoystick = null;
        if (userInputDevice != UserInputDevice.None)
        {
            relevantJoystick = joysticks[userInputDevice.getId()];
        }

        if (relevantJoystick == null || !relevantJoystick.isConnected())
        {
            ExceptionHelpers.Assert(TuningConstants.EXPECT_UNUSED_JOYSTICKS, "Unexpected user input device " + userInputDevice.toString());
            return false;
        }

        // find the appropriate button and grab the value from the relevant joystick
        UserInputDeviceButton relevantButton = this.getUserInputDeviceButton();

        if (relevantButton == UserInputDeviceButton.POV)
        {
            return relevantJoystick.getPOV() == this.getUserInputDevicePovValue();
        }
        else if (relevantButton == UserInputDeviceButton.ANALOG_AXIS_RANGE)
        {
            double value = relevantJoystick.getAxis(this.userInputDeviceAxis.Value);
            return
                value >= this.userInputDeviceAxisRangeMin &&
                value <= this.userInputDeviceAxisRangeMax;
        }
        else if (relevantButton != UserInputDeviceButton.NONE)
        {
            return relevantJoystick.getRawButton(relevantButton.Value);
        }
        else
        {
            return false;
        }
    }
}
