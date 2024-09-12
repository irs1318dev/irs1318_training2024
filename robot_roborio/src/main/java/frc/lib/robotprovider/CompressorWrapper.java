package frc.lib.robotprovider;

import edu.wpi.first.wpilibj.Compressor;

public class CompressorWrapper implements ICompressor
{
    private final Compressor wrappedObject;

    public CompressorWrapper(PneumaticsModuleType moduleType)
    {
        this.wrappedObject = new Compressor(PneumaticHubWrapper.getModuleType(moduleType));
    }

    public CompressorWrapper(int module, PneumaticsModuleType moduleType)
    {
        this.wrappedObject = new Compressor(module, PneumaticHubWrapper.getModuleType(moduleType));
    }

    public void enableAnalog(double minPressurePSI, double maxPressurePSI)
    {
        this.wrappedObject.enableAnalog(minPressurePSI, maxPressurePSI);
    }

    public void enableHybrid(double minPressurePSI, double maxPressurePSI)
    {
        this.wrappedObject.enableHybrid(minPressurePSI, maxPressurePSI);
    }

    public void enableDigital()
    {
        this.wrappedObject.enableDigital();
    }

    public double getPressure()
    {
        return this.wrappedObject.getPressure();
    }

    public void disable()
    {
        this.wrappedObject.disable();
    }
}
