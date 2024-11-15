package frc.lib.robotprovider;

import com.ctre.phoenix.sensors.*;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;

public class PigeonIMUWrapper implements IPigeonIMU
{
    private final PigeonIMU wrappedObject;

    private final String pigeonId;

    public PigeonIMUWrapper(int deviceNumber)
    {
        this.wrappedObject = new PigeonIMU(deviceNumber);
        this.pigeonId = String.format("Pigeon2 %d", deviceNumber);
    }

    public void getYawPitchRoll(double[] ypr_deg)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.getYawPitchRoll(ypr_deg),
            this.pigeonId,
            "PigeonIMU.getYawPitchRoll");
    }

    public void getRawGyro(double[] xyz_dps)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.getRawGyro(xyz_dps),
            this.pigeonId,
            "PigeonIMU.getRawGyro");
    }

    public void setYaw(double angleDeg)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.setYaw(angleDeg),
            this.pigeonId,
            "PigeonIMU.setYaw");
    }

    public PigeonState getState()
    {
        PigeonIMU.PigeonState state = this.wrappedObject.getState();
        return PigeonState.getValue(state.value);
    }

    public void enterTemperatureCalibrationMode()
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.enterCalibrationMode(CalibrationMode.Temperature),
            this.pigeonId,
            "PigeonIMU.enterTemperatureCalibrationMode");
    }
}