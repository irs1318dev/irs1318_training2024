package frc.lib.robotprovider;

import com.ctre.phoenix.led.*;

public class CANdleWrapper implements ICANdle
{
    private final CANdle wrappedObject;

    public CANdleWrapper(int deviceNumber)
    {
        this.wrappedObject = new CANdle(deviceNumber);
    }

    public CANdleWrapper(int deviceNumber, String canbus)
    {
        this.wrappedObject = new CANdle(deviceNumber, canbus);
    }

    public double getBusVoltage()
    {
        return this.wrappedObject.getBusVoltage();
    }

    public double get5VRailVoltage()
    {
        return this.wrappedObject.get5VRailVoltage();
    }

    public double getCurrent()
    {
        return this.wrappedObject.getCurrent();
    }

    public double getTemperature()
    {
        return this.wrappedObject.getTemperature();
    }

    public int getMaxSimultaneousAnimationCount()
    {
        return this.wrappedObject.getMaxSimultaneousAnimationCount();
    }

    public void configBrightnessScalar(double brightness)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.configBrightnessScalar(brightness),
            "CANdle.configBrightnessScalar");
    }

    public void configLEDType(CANdleLEDStripType type)
    {
        CANdle.LEDStripType stripType = CANdle.LEDStripType.GRB;
        switch (type)
        {
            case GRB:
                stripType = CANdle.LEDStripType.GRB;
                break;

            case RGB:
                stripType = CANdle.LEDStripType.RGB;
                break;

            case BRG:
                stripType = CANdle.LEDStripType.BRG;
                break;

            case GRBW:
                stripType = CANdle.LEDStripType.GRBW;
                break;

            case RGBW:
                stripType = CANdle.LEDStripType.RGBW;
                break;

            case BRGW:
                stripType = CANdle.LEDStripType.BRGW;
                break;
        }

        CTREStatusCodeHelper.printError(
            this.wrappedObject.configLEDType(stripType),
            "CANdle.configLEDType");
    }

    public void configLOSBehavior(boolean disableWhenLOS)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.configLOSBehavior(disableWhenLOS),
            "CANdle.configLOSBehavior");
    }

    public void configStatusLedState(boolean disableWhenRunning)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.configStatusLedState(disableWhenRunning),
            "CANdle.configStatusLedState");
    }

    public void configVBatOutput(CANdleVBatOutputMode mode)
    {
        CANdle.VBatOutputMode outputMode = CANdle.VBatOutputMode.Off;
        switch (mode)
        {
            case On:
                outputMode = CANdle.VBatOutputMode.On;
                break;

            case Off:
                outputMode = CANdle.VBatOutputMode.Off;
                break;

            case Modulated:
                outputMode = CANdle.VBatOutputMode.Modulated;
                break;
        }

        CTREStatusCodeHelper.printError(
            this.wrappedObject.configVBatOutput(outputMode),
            "CANdle.configVBatOutput");
    }

    public void modulateVBatOutput(double dutyCyclePercent)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.modulateVBatOutput(dutyCyclePercent),
            "CANdle.modulateVBatOutput");
    }

    public void setLEDs(int r, int g, int b)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.setLEDs(r, g, b),
            "CANdle.setLEDs");
    }

    public void setLEDs(int r, int g, int b, int w, int startIdx, int count)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.setLEDs(r, g, b, w, startIdx, count),
            "CANdle.setLEDs");
    }

    public void startTwinkleAnimation(int animSlot, int r, int g, int b, int w, double speed, int numLed, CANdleTwinklePercent divider, int ledOffset)
    {
        TwinkleAnimation.TwinklePercent twinkleDivider = TwinkleAnimation.TwinklePercent.Percent100;
        switch (divider)
        {
            case Percent100:
                twinkleDivider = TwinkleAnimation.TwinklePercent.Percent100;
                break;

            case Percent88:
                twinkleDivider = TwinkleAnimation.TwinklePercent.Percent88;
                break;

            case Percent76:
                twinkleDivider = TwinkleAnimation.TwinklePercent.Percent76;
                break;

            case Percent64:
                twinkleDivider = TwinkleAnimation.TwinklePercent.Percent64;
                break;

            case Percent42:
                twinkleDivider = TwinkleAnimation.TwinklePercent.Percent42;
                break;

            case Percent30:
                twinkleDivider = TwinkleAnimation.TwinklePercent.Percent30;
                break;

            case Percent18:
                twinkleDivider = TwinkleAnimation.TwinklePercent.Percent18;
                break;

            case Percent6:
                twinkleDivider = TwinkleAnimation.TwinklePercent.Percent6;
                break;
        }

        CTREStatusCodeHelper.printError(
            this.wrappedObject.animate(new TwinkleAnimation(r, g, b, w, speed, numLed, twinkleDivider, ledOffset), animSlot),
            "CANdle.startTwinkleAnimation");
    }

    public void startTwinkleOffAnimation(int animSlot, int r, int g, int b, int w, double speed, int numLed, CANdleTwinklePercent divider, int ledOffset)
    {
        TwinkleOffAnimation.TwinkleOffPercent twinkleDivider = TwinkleOffAnimation.TwinkleOffPercent.Percent100;
        switch (divider)
        {
            case Percent100:
                twinkleDivider = TwinkleOffAnimation.TwinkleOffPercent.Percent100;
                break;

            case Percent88:
                twinkleDivider = TwinkleOffAnimation.TwinkleOffPercent.Percent88;
                break;

            case Percent76:
                twinkleDivider = TwinkleOffAnimation.TwinkleOffPercent.Percent76;
                break;

            case Percent64:
                twinkleDivider = TwinkleOffAnimation.TwinkleOffPercent.Percent64;
                break;

            case Percent42:
                twinkleDivider = TwinkleOffAnimation.TwinkleOffPercent.Percent42;
                break;

            case Percent30:
                twinkleDivider = TwinkleOffAnimation.TwinkleOffPercent.Percent30;
                break;

            case Percent18:
                twinkleDivider = TwinkleOffAnimation.TwinkleOffPercent.Percent18;
                break;

            case Percent6:
                twinkleDivider = TwinkleOffAnimation.TwinkleOffPercent.Percent6;
                break;
        }

        CTREStatusCodeHelper.printError(
            this.wrappedObject.animate(new TwinkleOffAnimation(r, g, b, w, speed, numLed, twinkleDivider, ledOffset), animSlot),
            "CANdle.startTwinkleOffAnimation");
    }

    public void startStrobeAnimation(int animSlot, int r, int g, int b, int w, double speed, int numLed, int ledOffset)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.animate(new StrobeAnimation(r, g, b, w, speed, numLed, ledOffset), animSlot),
            "CANdle.startStrobeAnimation");
    }

    public void startSingleFadeAnimation(int animSlot, int r, int g, int b, int w, double speed, int numLed, int ledOffset)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.animate(new SingleFadeAnimation(r, g, b, w, speed, numLed, ledOffset), animSlot),
            "CANdle.startSingleFadeAnimation");
    }

    public void startRgbFadeAnimation(int animSlot, double brightness, double speed, int numLed, int ledOffset)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.animate(new RgbFadeAnimation(brightness, speed, numLed, ledOffset), animSlot),
            "CANdle.startRgbFaseAnimation");
    }

    public void startRainbowAnimation(int animSlot, double brightness, double speed, int numLed, boolean reverseDirection, int ledOffset)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.animate(new RainbowAnimation(brightness, speed, numLed, reverseDirection, ledOffset), animSlot),
            "CANdle.startRainbowAnimation");
    }

    public void startLarsonAnimation(int animSlot, int r, int g, int b, int w, double speed, int numLed, CANdleLarsonBounceMode mode, int size, int ledOffset)
    {
        LarsonAnimation.BounceMode bounceMode = LarsonAnimation.BounceMode.Front;
        switch (mode)
        {
            case Front:
                bounceMode = LarsonAnimation.BounceMode.Front;
                break;

            case Center:
                bounceMode = LarsonAnimation.BounceMode.Center;
                break;

            case Back:
                bounceMode = LarsonAnimation.BounceMode.Back;
                break;
        }

        CTREStatusCodeHelper.printError(
            this.wrappedObject.animate(new LarsonAnimation(r, g, b, w, speed, numLed, bounceMode, size, ledOffset), animSlot),
            "CANdle.startLarsonAnimation");
    }

    public void startFireAnimation(int animSlot, double brightness, double speed, int numLed, double sparking, double cooling, boolean reverseDirection, int ledOffset)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.animate(new FireAnimation(brightness, speed, numLed, sparking, cooling, reverseDirection, ledOffset), animSlot),
            "CANdle.startFireAnimation");
    }

    public void startColorFlowAnimation(int animSlot, int r, int g, int b, int w, double speed, int numLed, boolean forward, int ledOffset)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.animate(new ColorFlowAnimation(r, g, b, w, speed, numLed, forward ? ColorFlowAnimation.Direction.Forward : ColorFlowAnimation.Direction.Backward, ledOffset), animSlot),
            "CANdle.startColorFlowAnimation");
    }

    public void stopAnimation(int animSlot)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.clearAnimation(animSlot),
            "CANdle.stopAnimation");
    }
}