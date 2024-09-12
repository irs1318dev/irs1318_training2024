package frc.lib.robotprovider;

import java.util.Optional;
import java.util.OptionalInt;

import edu.wpi.first.wpilibj.DriverStation;

public class DriverStationWrapper implements IDriverStation
{
    public static final DriverStationWrapper Instance = new DriverStationWrapper();

    private DriverStationWrapper()
    {
    }

    @Override
    public String getEventName()
    {
        return DriverStation.getEventName();
    }

    @Override
    public Optional<Alliance> getAlliance()
    {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.isPresent())
        {
            return Optional.empty();
        }

        switch (alliance.get())
        {
            case Red:
                return Optional.of(Alliance.Red);

            case Blue:
                return Optional.of(Alliance.Blue);

            default:
                return Optional.empty();
        }
    }

    @Override
    public OptionalInt getLocation()
    {
        return DriverStation.getLocation();
    }

    @Override
    public int getMatchNumber()
    {
        return DriverStation.getMatchNumber();
    }

    @Override
    public MatchType getMatchType()
    {
        switch (DriverStation.getMatchType())
        {
            case Practice:
                return MatchType.Practice;

            case Qualification:
                return MatchType.Qualification;

            case Elimination:
                return MatchType.Elimination;

            default:
            case None:
                return MatchType.None;
        }
    }

    @Override
    public int getReplayNumber()
    {
        return DriverStation.getReplayNumber();
    }

    @Override
    public double getMatchTime()
    {
        return DriverStation.getMatchTime();
    }

    @Override
    public RobotMode getMode()
    {
        if (!DriverStation.isEnabled())
        {
            return RobotMode.Disabled;
        }
        else if (DriverStation.isAutonomous())
        {
            return RobotMode.Autonomous;
        }
        else if (DriverStation.isTest())
        {
            return RobotMode.Test;
        }
        else
        {
            return RobotMode.Teleop;
        }
    }

    @Override
    public boolean isFMSMode()
    {
        return DriverStation.isFMSAttached();
    }

    @Override
    public String getGameSpecificMessage()
    {
        return DriverStation.getGameSpecificMessage();
    }
}