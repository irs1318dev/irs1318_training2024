package frc.robot.mechanisms;

import frc.lib.mechanisms.*;
import frc.lib.robotprovider.Pose2d;

public interface IDriveTrainMechanism extends IMechanism
{
    Pose2d getPose();
    double getPositionX();
    double getPositionY();
    double[] getModuleTurnInPlaceAngles();
    double[] getDriveMotorPositions();
}
