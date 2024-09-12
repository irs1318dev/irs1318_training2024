package frc.lib.robotprovider;

import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.littletonrobotics.junction.networktables.LoggedDashboardString;

import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AKNetworkTableProvider implements INetworkTableProvider
{
    @Override
    public void startShuffleboardRecording()
    {
        Shuffleboard.startRecording();
    }

    @Override
    public void stopShuffleboardRecording()
    {
        Shuffleboard.stopRecording();
    }

    @Override
    public IIntegerSubscriber getIntegerSlider(String title, int initialValue)
    {
        return this.getIntegerSubscriber(title, initialValue);
    }

    @Override
    public IDoubleSubscriber getNumberSlider(String title, double initialValue)
    {
        return this.getDoubleSubscriber(title, initialValue);
    }

    @Override
    public IBooleanSubscriber getCheckbox(String title, boolean initialValue)
    {
        return this.getBooleanSubscriber(title, initialValue);
    }

    @Override
    public <V> ISendableChooser<V> getSendableChooser(String name)
    {
        return new AKSendableChooserWrapper<V>(name);
    }

    @Override
    public IDoubleSubscriber getDoubleSubscriber(String key)
    {
        return this.getDoubleSubscriber(key, 0.0);
    }

    @Override
    public IDoubleSubscriber getDoubleSubscriber(String key, double defaultValue)
    {
        LoggedDashboardNumber number = new LoggedDashboardNumber(key, defaultValue);
        return new AKDoubleSubscriberWrapper(number);
    }

    @Override
    public IBooleanSubscriber getBooleanSubscriber(String key)
    {
        return this.getBooleanSubscriber(key, false);
    }

    @Override
    public IBooleanSubscriber getBooleanSubscriber(String key, boolean defaultValue)
    {
        LoggedDashboardBoolean bool = new LoggedDashboardBoolean(key, defaultValue);
        return new AKBooleanSubscriberWrapper(bool);
    }

    @Override
    public IIntegerSubscriber getIntegerSubscriber(String key)
    {
        return this.getIntegerSubscriber(key, 0);
    }

    @Override
    public IIntegerSubscriber getIntegerSubscriber(String key, int defaultValue)
    {
        LoggedDashboardNumber number = new LoggedDashboardNumber(key, defaultValue);
        return new AKIntegerSubscriberWrapper(number);
    }

    @Override
    public IStringSubscriber getStringSubscriber(String key)
    {
        return this.getStringSubscriber(key, null);
    }

    @Override
    public IStringSubscriber getStringSubscriber(String key, String defaultValue)
    {
        LoggedDashboardString str = new LoggedDashboardString(key, defaultValue);
        return new AKStringSubscriberWrapper(str);
    }
}