package frc.robot.mechanisms;

import frc.robot.*;
import frc.lib.driver.IDriver;
import frc.lib.helpers.Helpers;
import frc.lib.mechanisms.IMechanism;
import frc.lib.mechanisms.LoggingManager;
import frc.lib.robotprovider.*;
import frc.robot.driver.*;

import java.util.List;
import java.util.Optional;

import com.google.inject.Inject;
import com.google.inject.Singleton;

/**
 * Offboard Vision manager.
 */
@Singleton
public class OffboardVisionManager implements IMechanism
{
    public static final DigitalOperation[] PossibleVisionOperations =
    {
        DigitalOperation.VisionFindAnyAprilTag,
        DigitalOperation.VisionFindSpecificAprilTag,
        DigitalOperation.VisionFindAbsolutePosition,
    };

    public static final List<DigitalOperation> PossibleFrontVisionOperations =
        List.of(
            DigitalOperation.VisionFindSpecificAprilTag,
            DigitalOperation.VisionFindAbsolutePosition);

    private final IDriver driver;
    private final ILogger logger;

    private final INetworkTableProvider networkTable;
    private final IDriverStation ds;

    private IDoubleSubscriber atXOffsetSubscriber;
    private IDoubleSubscriber atYOffsetSubscriber;
    private IDoubleSubscriber atZOffsetSubscriber;
    private IDoubleSubscriber atYawSubscriber;
    private IDoubleSubscriber atPitchSubscriber;
    private IDoubleSubscriber atRollSubscriber;
    private IIntegerSubscriber atIdSubscriber;

    private IDoubleSubscriber absXOffsetSubscriber;
    private IDoubleSubscriber absYOffsetSubscriber;
    private IDoubleSubscriber absZOffsetSubscriber;
    private IDoubleSubscriber absRollAngleSubscriber;
    private IDoubleSubscriber absPitchAngleSubscriber;
    private IDoubleSubscriber absYawAngleSubscriber;
    private IDoubleSubscriber absTagIdSubscriber;
    private IDoubleSubscriber absDecisionMarginSubscriber;
    private IDoubleSubscriber absErrorSubscriber;

    private IDoubleSubscriber heartbeatSubscriber;

    private Double atXOffset;
    private Double atYOffset;
    private Double atZOffset;
    private Double atYaw;
    private Double atPitch;
    private Double atRoll;
    private Integer atId;

    private Double absXOffset;
    private Double absYOffset;
    private Double absZOffset;
    private Double absRollAngle;
    private Double absPitchAngle;
    private Double absYawAngle;
    private Integer absTagId;
    private Double absDecisionMargin;
    private Double absError;

    private int prevMode;
    private List<Integer> prevTargets;

    private int missedHeartbeats;
    private double prevHeartbeat;

    /**
     * Initializes a new OffboardVisionManager
     * @param driver for obtaining operations
     * @param logger for logging to smart dashboard
     * @param provider for obtaining electronics objects
     */
    @Inject
    public OffboardVisionManager(IDriver driver, LoggingManager logger, IRobotProvider provider)
    {
        this.driver = driver;
        this.logger = logger;

        this.networkTable = provider.getNetworkTableProvider();
        this.atXOffsetSubscriber = this.networkTable.getDoubleSubscriber("at.xOffset", TuningConstants.MAGIC_NULL_VALUE);
        this.atYOffsetSubscriber = this.networkTable.getDoubleSubscriber("at.yOffset", TuningConstants.MAGIC_NULL_VALUE);
        this.atZOffsetSubscriber = this.networkTable.getDoubleSubscriber("at.zOffset", TuningConstants.MAGIC_NULL_VALUE);
        this.atYawSubscriber = this.networkTable.getDoubleSubscriber("at.yawAngle", TuningConstants.MAGIC_NULL_VALUE);
        this.atPitchSubscriber = this.networkTable.getDoubleSubscriber("at.pitchAngle", TuningConstants.MAGIC_NULL_VALUE);
        this.atRollSubscriber = this.networkTable.getDoubleSubscriber("at.rollAngle", TuningConstants.MAGIC_NULL_VALUE);
        this.atIdSubscriber = this.networkTable.getIntegerSubscriber("at.tagId", (int) TuningConstants.MAGIC_NULL_VALUE);

        this.absXOffsetSubscriber = this.networkTable.getDoubleSubscriber("abs.xOffset", TuningConstants.MAGIC_NULL_VALUE);
        this.absYOffsetSubscriber = this.networkTable.getDoubleSubscriber("abs.yOffset", TuningConstants.MAGIC_NULL_VALUE);
        this.absZOffsetSubscriber = this.networkTable.getDoubleSubscriber("abs.zOffset", TuningConstants.MAGIC_NULL_VALUE);
        this.absRollAngleSubscriber = this.networkTable.getDoubleSubscriber("abs.rollAngle", TuningConstants.MAGIC_NULL_VALUE);
        this.absPitchAngleSubscriber = this.networkTable.getDoubleSubscriber("abs.pitchAngle", TuningConstants.MAGIC_NULL_VALUE);
        this.absYawAngleSubscriber = this.networkTable.getDoubleSubscriber("abs.yawAngle", TuningConstants.MAGIC_NULL_VALUE);
        this.absTagIdSubscriber = this.networkTable.getDoubleSubscriber("abs.tagId", TuningConstants.MAGIC_NULL_VALUE);
        this.absDecisionMarginSubscriber = this.networkTable.getDoubleSubscriber("abs.decisionMargin", TuningConstants.MAGIC_NULL_VALUE);
        this.absErrorSubscriber = this.networkTable.getDoubleSubscriber("abs.error", TuningConstants.MAGIC_NULL_VALUE);

        this.heartbeatSubscriber = this.networkTable.getDoubleSubscriber("v.heartbeat", 0);

        this.ds = provider.getDriverStation();

        this.atXOffset = null;
        this.atYOffset = null;
        this.atZOffset = null;
        this.atYaw = null;
        this.atPitch = null;
        this.atRoll = null;
        this.atId = null;

        this.prevMode = 0;
        this.prevTargets = null;

        this.missedHeartbeats = 0;
        this.prevHeartbeat = 0.0;
    }

    /**
     * read all of the sensors for the mechanism that we will use in macros/autonomous mode and record their values
     */
    @Override
    public void readSensors()
    {
        double atXOffset = this.atXOffsetSubscriber.get();
        double atYOffset = this.atYOffsetSubscriber.get();
        double atZOffset = this.atZOffsetSubscriber.get();
        double atYaw = this.atYawSubscriber.get();
        double atPitch = this.atPitchSubscriber.get();
        double atRoll = this.atRollSubscriber.get();
        int atId = (int)this.atIdSubscriber.get();

        double absXOffset = this.absXOffsetSubscriber.get();
        double absYOffset = this.absYOffsetSubscriber.get();
        double absZOffset = this.absZOffsetSubscriber.get();
        double absRollAngle = this.absRollAngleSubscriber.get();
        double absPitchAngle = this.absPitchAngleSubscriber.get();
        double absYawAngle = this.absYawAngleSubscriber.get();
        int absTagId = (int)this.absTagIdSubscriber.get();
        double absDecisionMargin = this.absDecisionMarginSubscriber.get();
        double absError = this.absErrorSubscriber.get();

        double newHeartbeat = this.heartbeatSubscriber.get();
        if (!Helpers.RoughEquals(this.prevHeartbeat, newHeartbeat, 0.5))
        {
            this.missedHeartbeats = 0;
        }
        else
        {
            this.missedHeartbeats++;
        }

        this.prevHeartbeat = newHeartbeat;
        this.logger.logNumber(LoggingKey.OffboardVisionMissedHeartbeats, this.missedHeartbeats);

        boolean missedHeartbeatExceedsThreshold = this.missedHeartbeats > TuningConstants.VISION_MISSED_HEARTBEAT_THRESHOLD;
        this.logger.logBoolean(LoggingKey.OffboardVisionExcessiveMissedHeartbeats, missedHeartbeatExceedsThreshold);

        // reset if we couldn't find the april tag
        this.atXOffset = null;
        this.atYOffset = null;
        this.atZOffset = null;
        this.atYaw = null;
        this.atPitch = null;
        this.atRoll = null;
        this.atId = null;

        this.absXOffset = null;
        this.absYOffset = null;
        this.absZOffset = null;
        this.absRollAngle = null;
        this.absPitchAngle = null;
        this.absYawAngle = null;
        this.absTagId = null;
        this.absDecisionMargin = null;
        this.absError = null;

        if (!missedHeartbeatExceedsThreshold)
        {
            switch (this.prevMode)
            {
                case 1:
                    if (atXOffset != TuningConstants.MAGIC_NULL_VALUE &&
                        atYOffset != TuningConstants.MAGIC_NULL_VALUE &&
                        atZOffset != TuningConstants.MAGIC_NULL_VALUE &&
                        (this.prevTargets == null || this.prevTargets.contains(atId)))
                    {
                        this.atXOffset = atXOffset;
                        this.atYOffset = atYOffset;
                        this.atZOffset = atZOffset;
                        this.atYaw = atYaw;
                        this.atPitch = atPitch;
                        this.atRoll = atRoll;
                        this.atId = atId;
                    }

                    break;

                case 3:
                    if (absXOffset != TuningConstants.MAGIC_NULL_VALUE &&
                        absYOffset != TuningConstants.MAGIC_NULL_VALUE &&
                        absZOffset != TuningConstants.MAGIC_NULL_VALUE)
                    {
                        this.absXOffset = absXOffset;
                        this.absYOffset = absYOffset;
                        this.absZOffset = absZOffset;
                        this.absRollAngle = absRollAngle;
                        this.absPitchAngle = absPitchAngle;
                        this.absYawAngle = absYawAngle;
                        this.absTagId = absTagId;
                        this.absDecisionMargin = absDecisionMargin;
                        this.absError = absError;
                    }

                case 0:
                default:
                    break;
            }
        }

        this.logger.logNumber(LoggingKey.OffboardVisionAprilTagXOffset, this.atXOffset);
        this.logger.logNumber(LoggingKey.OffboardVisionAprilTagYOffset, this.atYOffset);
        this.logger.logNumber(LoggingKey.OffboardVisionAprilTagZOffset, this.atZOffset);
        this.logger.logNumber(LoggingKey.OffboardVisionAprilTagYaw, this.atYaw);
        this.logger.logNumber(LoggingKey.OffboardVisionAprilTagPitch, this.atPitch);
        this.logger.logNumber(LoggingKey.OffboardVisionAprilTagRoll, this.atRoll);
        this.logger.logInteger(LoggingKey.OffboardVisionAprilTagId, this.atId);
    }

    @Override
    public void update(RobotMode mode)
    {
        boolean enableVision = !this.driver.getDigital(DigitalOperation.VisionForceDisable);
        boolean enableVideoStream = mode == RobotMode.Test || this.driver.getDigital(DigitalOperation.VisionEnableStream);
        boolean enableAny = this.driver.getDigital(DigitalOperation.VisionFindAnyAprilTag);
        boolean enableSpecific = this.driver.getDigital(DigitalOperation.VisionFindSpecificAprilTag);
        boolean enableAbsolutePosition = this.driver.getDigital(DigitalOperation.VisionFindAbsolutePosition);

        Optional<Alliance> alliance = this.ds.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;

        List<Integer> desiredTargets = null;
        String desiredTargetsString = "";
        int visionProcessingMode = 0;
        if (enableVision)
        {
            if (enableAny || enableSpecific)
            {
                visionProcessingMode = 1;
            }
            else if (enableAbsolutePosition)
            {
                visionProcessingMode = 3;
            }

            if (enableSpecific)
            {
                if (isRed)
                {
                    desiredTargets = TuningConstants.VISION_SPEAKER_RED_APRILTAGS;
                    desiredTargetsString = TuningConstants.VISION_SPEAKER_RED_STRING;
                }
                else
                {
                    desiredTargets = TuningConstants.VISION_SPEAKER_BLUE_APRILTAGS;
                    desiredTargetsString = TuningConstants.VISION_SPEAKER_BLUE_STRING;
                }
            }
        }

        this.prevMode = visionProcessingMode;
        this.prevTargets = desiredTargets;
        this.logger.logBoolean(LoggingKey.OffboardVisionEnableStream, enableVideoStream);
        this.logger.logInteger(LoggingKey.OffboardVisionProcessingMode, visionProcessingMode);
        this.logger.logString(LoggingKey.OffboardVisionDesiredTarget, desiredTargetsString);
    }

    @Override
    public void stop()
    {
        this.prevMode = 0;
        this.prevTargets = null;
        this.logger.logBoolean(LoggingKey.OffboardVisionEnableStream, false);
        this.logger.logInteger(LoggingKey.OffboardVisionProcessingMode, 0);
        this.logger.logString(LoggingKey.OffboardVisionDesiredTarget, "");
    }

    public Double getAprilTagXOffset()
    {
        return this.atXOffset;
    }

    public Double getAprilTagYOffset()
    {
        return this.atYOffset;
    }

    public Double getAprilTagZOffset()
    {
        return this.atZOffset;
    }

    public Double getAprilTagYaw()
    {
        return this.atYaw;
    }

    public Double getAprilTagPitch()
    {
        return this.atPitch;
    }

    public Double getAprilTagRoll()
    {
        return this.atRoll;
    }

    public Integer getAprilTagId()
    {
        return this.atId;
    }

    public Double getAbsolutePositionX()
    {
        return this.absXOffset;
    }

    public Double getAbsolutePositionY()
    {
        return this.absYOffset;
    }

    public Double getAbsolutePositionZ()
    {
        return this.absZOffset;
    }

    public Double getAbsolutePositionYaw()
    {
        return this.absYawAngle;
    }
}
