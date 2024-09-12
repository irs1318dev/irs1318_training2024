package frc.lib.robotprovider;

import javax.inject.Inject;
import javax.inject.Singleton;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LoggingKey;

/**
 * Logger that logs current values to a dashboard.
 *
 */
@Singleton
public class SmartDashboardLogger implements ISmartDashboardLogger
{
    private int loggingCounter;

    @Inject
    public SmartDashboardLogger()
    {
        this.loggingCounter = 0;
    }

    /**
     * Write a boolean to the smart dashboard
     * @param key to write to
     * @param value to write
     */
    @Override
    public void logBoolean(LoggingKey key, boolean value)
    {
        if ((this.loggingCounter % key.loggingFrequency) == 0)
        {
            if (SmartDashboard.getBoolean(key.value, !value) != value)
            {
                SmartDashboard.putBoolean(key.value, value);
            }
        }
    }

    /**
     * Write a boolean array to the smart dashboard
     * @param key to write to
     * @param value to write
     */
    @Override
    public void logBooleanArray(LoggingKey key, boolean[] value)
    {
        if ((this.loggingCounter % key.loggingFrequency) == 0)
        {
            SmartDashboard.putBooleanArray(key.value, value);
        }
    }

    /**
     * Write a number (double) to the smart dashboard
     * @param key to write to
     * @param value to write
     */
    @Override
    public void logNumber(LoggingKey key, double value)
    {
        if ((this.loggingCounter % key.loggingFrequency) == 0)
        {
            if (SmartDashboard.getNumber(key.value, value + 0.5) != value)
            {
                SmartDashboard.putNumber(key.value, value);
            }
        }
    }

    /**
     * Write a number (double) to the smart dashboard
     * @param key to write to
     * @param value to write
     */
    @Override
    public void logNumber(LoggingKey key, Double value)
    {
        String valueString = "N/A";
        if (value != null)
        {
            valueString = String.valueOf(value);
        }

        this.logString(key, valueString);
    }

    /**
     * Write a number (integer) to the smart dashboard
     * @param key to write to
     * @param value to write
     */
    @Override
    public void logInteger(LoggingKey key, int value)
    {
        this.logInteger(key, value, null);
    }

    /**
     * Write a number (integer) to the smart dashboard
     * @param key to write to
     * @param value to write
     */
    @Override
    public void logInteger(LoggingKey key, Integer value)
    {
        String valueString = "N/A";
        if (value != null)
        {
            valueString = String.valueOf(value);
        }

        this.logString(key, valueString);
    }

    /**
     * Write a number (integer) to the smart dashboard
     * @param key to write to
     * @param value to write
     * @param formatString to use
     */
    @Override
    public void logInteger(LoggingKey key, int value, String formatString)
    {
        if ((this.loggingCounter % key.loggingFrequency) == 0)
        {
            if (SmartDashboard.getNumber(key.value, value + 0.5) != value)
            {
                SmartDashboard.putNumber(key.value, value);
            }
        }
    }

    /**
     * Write a string to the smart dashboard
     * @param key to write to
     * @param value to write
     */
    @Override
    public void logString(LoggingKey key, String value)
    {
        if ((this.loggingCounter % key.loggingFrequency) == 0)
        {
            if (value == null)
            {
                value = "";
            }

            String currValue = SmartDashboard.getString(key.value, "");
            if ((value != null && currValue != null && !currValue.equals(value)) ||
                (value != null) != (currValue != null))
            {
                SmartDashboard.putString(key.value, value);
            }
        }
    }

    /**
     * Update the log, if appropriate..
     */
    @Override
    public void update()
    {
        this.loggingCounter++;
    }

    /**
     * Flush the output stream, if appropriate..
     */
    @Override
    public void flush()
    {
    }
}
