package frc.robot;

import javax.inject.Singleton;

import frc.lib.driver.*;
import frc.lib.mechanisms.MechanismManager;
import frc.lib.robotprovider.*;
import frc.robot.driver.*;

import com.google.inject.AbstractModule;
import com.google.inject.Injector;
import com.google.inject.Provides;

public class RobotModule extends AbstractModule
{
    @Override
    protected void configure()
    {
        this.bind(IDriver.class).to(Driver.class);
        this.bind(IRobotProvider.class).to(RobotProvider.class);
        this.bind(ITimer.class).to(TimerWrapper.class);
        this.bind(IButtonMap.class).to(ButtonMap.class);
        this.bind(IFile.class).to(FileWrapper.class);

        if (TuningConstants.USE_ADVANTAGE_KIT)
        {
            this.bind(ISmartDashboardLogger.class).to(AdvantageKitLogger.class);
        }
        else
        {
            this.bind(ISmartDashboardLogger.class).to(SmartDashboardLogger.class);
        }
    }

    @Singleton
    @Provides
    public MechanismManager getMechanismManager(Injector injector)
    {
        return new MechanismManager(SettingsManager.getActiveMechanisms(injector));
    }
}
