package frc.lib.robotprovider;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class AKDoubleSubscriberWrapper extends AKNumberSubscriberWrapperBase implements IDoubleSubscriber
{
    AKDoubleSubscriberWrapper(LoggedDashboardNumber number)
    {
        super(number);
    }

    @Override
    public double get()
    {
        return this.number.get();
    }
}
