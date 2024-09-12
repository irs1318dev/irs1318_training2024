package frc.robot;
import java.util.Optional;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import frc.lib.robotprovider.Alliance;
import frc.lib.robotprovider.IDriverStation;
import frc.lib.robotprovider.IRobotProvider;
import frc.lib.robotprovider.Point2d;

@Singleton
public class AutonLocManager
{

    private boolean isRed;
    private IDriverStation driverStation;

    public Point2d P1;
    public Point2d P2;
    public Point2d P2A;
    public Point2d P2After;
    public Point2d P2Vision;
    public Point2d P3;
    public Point2d P4;
    public Point2d P5;
    public Point2d P5M;
    public Point2d P6;
    public Point2d P6M;
    public Point2d P7;
    public Point2d P7M;
    public Point2d P8;
    public Point2d P8M;
    public Point2d P9;
    public Point2d P9M;
    public Point2d P10;
    public Point2d P10M;
    public Point2d P11;
    public Point2d P11M;
    public Point2d P12;
    public Point2d P12M;
    public Point2d P13;
    public Point2d P14;
    public Point2d P15;
    public Point2d P16;
    public Point2d P17;
    public Point2d P18;
    public Point2d P19;
    public Point2d P19S;
    public Point2d P20;
    public Point2d P21;
    public Point2d PSNK1;
    public Point2d PSNK2;
    public Point2d C1;
    public Point2d C2;
    public Point2d C3;

    public AutonLocManager(boolean isRed)
    {
        this.isRed = isRed;
        this.setValues();
    }

    @Inject
    public AutonLocManager(IRobotProvider provider) 
    {
        this.driverStation = provider.getDriverStation();
    }

    public void updateAlliance()
    {
        Optional<Alliance> alliance = driverStation.getAlliance();
        this.isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
        this.setValues();
    }

    public boolean getRedUpdateAlliance()
    {
        Optional<Alliance> alliance = driverStation.getAlliance();
        this.isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
        this.setValues();
        return isRed;
    }

    public double getOrientationOrHeading(double orientationOrHeading)
    {
        return AutonLocManager.getOrientationOrHeading(this.isRed, orientationOrHeading);
    }

    public boolean getIsRed()
    {
        return this.isRed;
    }

    private void setValues()
    {
        //Red is positive
        //Blue is negative

        //Y field length: 323in
        //X field length: 653in

        this.P1 = new Point2d(AutonLocManager.getXPosition(this.isRed, 326.5 - HardwareConstants.ROBOT_HALF_FRAME_PERIMETER), 64 + HardwareConstants.ROBOT_HALF_FRAME_PERIMETER);
        this.P2 = new Point2d(AutonLocManager.getXPosition(this.isRed, 289 + 4.818761855), 198 - 23.5416793); //236.7 degrees orientation
        this.P2After = new Point2d(AutonLocManager.getXPosition(this.isRed, 289 + 4.818761855 - 8.0), 198 - 23.5416793 + 8.0); //236.7 degrees orientation
        this.P2Vision = new Point2d(AutonLocManager.getXPosition(this.isRed, 289 + 4.818761855 - 60.0), 198 - 23.5416793 - 30); //236.7 degrees orientation
        this.P2A = new Point2d(AutonLocManager.getXPosition(this.isRed, 289 + 4.818761855) , 238 + 27.54); //123.3 degrees orientation
        this.P3 = new Point2d(AutonLocManager.getXPosition(this.isRed, 250.5 + HardwareConstants.ROBOT_HALF_FRAME_PERIMETER), 306 - HardwareConstants.ROBOT_HALF_FRAME_PERIMETER);
        this.P4 = new Point2d(AutonLocManager.getXPosition(this.isRed, 288 - HardwareConstants.ROBOT_HALF_FRAME_PERIMETER), 239 - HardwareConstants.ROBOT_HALF_FRAME_PERIMETER);
        this.P5 = new Point2d(AutonLocManager.getXPosition(this.isRed, 212 + 8), 160);//212), 162);
        this.P5M = new Point2d(AutonLocManager.getXPosition(this.isRed, 220 + 20), 160);
        this.P6 = new Point2d(AutonLocManager.getXPosition(this.isRed, 212), 215);
        this.P6M = new Point2d(AutonLocManager.getXPosition(this.isRed, 212 + 20), 215);
        this.P7 = new Point2d(AutonLocManager.getXPosition(this.isRed, 212), 276);
        this.P7M = new Point2d(AutonLocManager.getXPosition(this.isRed, 212 /*p7*/ + 28), 276); //p7
        this.P8 = new Point2d(AutonLocManager.getXPosition(this.isRed, -15), 32.5);
        this.P8M = new Point2d(AutonLocManager.getXPosition(this.isRed, -15/*p8 */ + 20), 32.5); 
        this.P9 = new Point2d(AutonLocManager.getXPosition(this.isRed, -15), 120.64);
        this.P9M = new Point2d(AutonLocManager.getXPosition(this.isRed, -15 /*p9 */ + 20), 120.64);
        this.P10 = new Point2d(AutonLocManager.getXPosition(this.isRed, 0), 161.64);
        this.P10M = new Point2d(AutonLocManager.getXPosition(this.isRed, 0 /*p10*/ + 20), 161.64);
        this.P11 = new Point2d(AutonLocManager.getXPosition(this.isRed, 0), 237.64);
        this.P11M = new Point2d(AutonLocManager.getXPosition(this.isRed, 0 /*p11 */ + 20), 237.64);
        this.P12 = new Point2d(AutonLocManager.getXPosition(this.isRed, 0), 283.64);
        this.P12M = new Point2d(AutonLocManager.getXPosition(this.isRed, 0 + /*p12 */ 20), 283.64);

        //ToDO fix 13 14
        this.P13 = new Point2d(AutonLocManager.getXPosition(this.isRed, 0), 77.0); //93.154754 - HardwareConstants.ROBOT_HALF_FRAME_PERIMETER);
        this.P14 = new Point2d(AutonLocManager.getXPosition(this.isRed, 0), 231.777 + HardwareConstants.ROBOT_HALF_FRAME_PERIMETER);

        //ToDo : add p15, p16, p17
        this.P15 = new Point2d(0,0); //Might use
        this.P16 = new Point2d(AutonLocManager.getXPosition(this.isRed, 172.955),  93.154754 - HardwareConstants.ROBOT_HALF_FRAME_PERIMETER);
        this.P17 = new Point2d(AutonLocManager.getXPosition(this.isRed, 172.955),  250.777 + HardwareConstants.ROBOT_HALF_FRAME_PERIMETER);
        this.P18 = new Point2d(AutonLocManager.getXPosition(this.isRed, 115 + 21 + HardwareConstants.ROBOT_HALF_FRAME_PERIMETER), 162);
        this.P19 = new Point2d(AutonLocManager.getXPosition(this.isRed, 204), 77); // 107
        this.P19S = new Point2d(AutonLocManager.getXPosition(this.isRed, 204), 107);
        this.P20 = new Point2d(AutonLocManager.getXPosition(this.isRed, 190), 217);
        this.P21 = new Point2d(AutonLocManager.getXPosition(this.isRed, 72), 0);

        // Points for sneaking
        this.PSNK1 = new Point2d(AutonLocManager.getXPosition(this.isRed, 235), 310.0);
        this.PSNK2 = new Point2d(AutonLocManager.getXPosition(this.isRed, 173), 310.0);

        //Climber
        this.C1 = new Point2d(AutonLocManager.getXPosition(this.isRed, 1318), 1318);
        this.C2 = new Point2d(AutonLocManager.getXPosition(this.isRed, 1318), 1318);
        this.C3 = new Point2d(AutonLocManager.getXPosition(this.isRed, 1318), 1318);

        
    }

    private static double getOrientationOrHeading(boolean isRed, double orientationOrHeading)
    {
        if (isRed)
        {
            return orientationOrHeading;
        }
        else
        {
            return 180.0 - orientationOrHeading;
        }
    }

    private static double getXPosition(boolean isRed, double position)
    {
        if (isRed)
        {
            return position;
        }
        else
        {
            return position * -1.0;
        }
    }
}
