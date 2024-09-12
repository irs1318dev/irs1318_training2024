package frc.lib.robotprovider;

import org.littletonrobotics.junction.networktables.LoggedDashboardString;

public class AKStringSubscriberWrapper implements IStringSubscriber
{
    private final LoggedDashboardString str;

    AKStringSubscriberWrapper(LoggedDashboardString str)
    {
        this.str = str;
    }

    @Override
    public String get()
    {
        return this.str.get();
    }
}
