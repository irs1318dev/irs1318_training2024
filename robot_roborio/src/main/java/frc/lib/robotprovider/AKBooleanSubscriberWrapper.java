package frc.lib.robotprovider;

import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

public class AKBooleanSubscriberWrapper implements IBooleanSubscriber
{
    private final LoggedDashboardBoolean bool;

    AKBooleanSubscriberWrapper(LoggedDashboardBoolean bool)
    {
        this.bool = bool;
    }

    @Override
    public boolean get()
    {
        return this.bool.get();
    }
}
