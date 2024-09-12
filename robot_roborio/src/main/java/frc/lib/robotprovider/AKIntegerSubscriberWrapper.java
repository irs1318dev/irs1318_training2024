package frc.lib.robotprovider;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class AKIntegerSubscriberWrapper extends AKNumberSubscriberWrapperBase implements IIntegerSubscriber
{
    AKIntegerSubscriberWrapper(LoggedDashboardNumber number)
    {
        super(number);
    }

    @Override
    public long get()
    {
        return (long)this.number.get();
    }
}
