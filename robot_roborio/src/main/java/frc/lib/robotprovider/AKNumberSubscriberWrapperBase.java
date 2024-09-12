package frc.lib.robotprovider;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class AKNumberSubscriberWrapperBase
{
    protected final LoggedDashboardNumber number;

    AKNumberSubscriberWrapperBase(LoggedDashboardNumber number)
    {
        this.number = number;
    }
}
