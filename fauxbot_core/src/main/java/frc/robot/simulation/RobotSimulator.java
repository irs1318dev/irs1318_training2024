package frc.robot.simulation;

import java.util.HashMap;
import java.util.Map;

import frc.lib.robotprovider.*;
import space.earlygrey.shapedrawer.ShapeDrawer;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Pixmap.Format;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g2d.Batch;
import com.badlogic.gdx.graphics.g2d.TextureRegion;
import com.google.inject.Inject;
import com.google.inject.Singleton;

@Singleton
public class RobotSimulator extends SimulatorBase
{
    // private static final FauxbotSensorConnection EncoderAChannel = new FauxbotSensorConnection(FauxbotSensorConnection.SensorConnector.DigitalInput, FauxbotEncoder.class, 0);
    // private static final FauxbotSensorConnection EncoderBChannel = new FauxbotSensorConnection(FauxbotSensorConnection.SensorConnector.DigitalInput, FauxbotEncoder.class, 1);
    // private static final FauxbotActuatorConnection MotorChannel = new FauxbotActuatorConnection(FauxbotActuatorConnection.ActuatorConnector.PWM, 0);

    private final FauxbotSensorConnection[] sensors =
        new FauxbotSensorConnection[]
        {
            // RobotSimulator.EncoderAChannel,
            // RobotSimulator.EncoderBChannel,
        };

    private final FauxbotActuatorConnection[] actuators =
        new FauxbotActuatorConnection[]
        {
            // RobotSimulator.MotorChannel,
        };

    @SuppressWarnings("serial")
    private final Map<FauxbotSensorConnection, String> sensorNameMap = new HashMap<FauxbotSensorConnection, String>()
    {
        {
            // this.put(RobotSimulator.EncoderAChannel, "Elevator encoder");
            // this.put(RobotSimulator.EncoderBChannel, "Elevator encoder");
        }
    };

    @SuppressWarnings("serial")
    private final Map<FauxbotActuatorConnection, String> motorNameMap = new HashMap<FauxbotActuatorConnection, String>()
    {
        {
            // this.put(RobotSimulator.MotorChannel, "Elevator motor");
        }
    };

    // private static final double[] FloorHeightPercentages = new double[] { 0.0, 0.2, 0.4, 0.6, 0.8 };
    // private static final double PercentageAllowance = 0.01;
    // private static final float ElevatorCarWidth = 30.0f;
    // private static final float ElevatorCarHeight = 70.0f;

    // private static final float FrameDimension = 400.0f;
    // private static final float ElevatorMinHeight = 0.0f;
    // private static final float ElevatorMaxHeight = 250.0f;

    // private static final double MotorStrength = 700.0;
    // private static final double Gravity = -386.0;

    // private static final double ElevatorMinVelocity = -20.0;
    // private static final double ElevatorMaxVelocity = 20.0;

    // private Texture elevatorPassenger;
    // private Texture drawerTexture;

    // private double prevHeight;
    // private double prevVelocity;

    @Inject
    public RobotSimulator()
    {
        // this.elevatorPassenger = new Texture(Gdx.files.internal("images/stickFigure.png"));

        // Pixmap pixmap = new Pixmap(1, 1, Format.RGBA8888);
        // pixmap.setColor(Color.WHITE);
        // pixmap.drawPixel(0, 0);
        // this.drawerTexture = new Texture(pixmap);
        // pixmap.dispose();

        // this.prevHeight = 0.0;
        // this.prevVelocity = 0.0;

        // this.setSize(RobotSimulator.FrameDimension, RobotSimulator.FrameDimension);
    }

    @Override
    public FauxbotSensorConnection[] getSensors()
    {
        return this.sensors;
    }

    @Override
    public FauxbotActuatorConnection[] getActuators()
    {
        return this.actuators;
    }

    @Override
    public String getSensorName(FauxbotSensorConnection connection)
    {
        if (this.sensorNameMap.containsKey(connection))
        {
            return this.sensorNameMap.get(connection);
        }

        return "Sensor " + connection;
    }

    @Override
    public double getSensorMin(FauxbotSensorConnection connection)
    {
        return -1.0; // RobotSimulator.ElevatorMinHeight;
    }

    @Override
    public double getSensorMax(FauxbotSensorConnection connection)
    {
        return 1.0; // RobotSimulator.ElevatorMaxHeight;
    }

    @Override
    public String getActuatorName(FauxbotActuatorConnection connection)
    {
        if (this.motorNameMap.containsKey(connection))
        {
            return this.motorNameMap.get(connection);
        }

        return "Motor " + connection;
    }

    @Override
    public double getMotorMin(FauxbotActuatorConnection connection)
    {
        return -1.0;
    }

    @Override
    public double getMotorMax(FauxbotActuatorConnection connection)
    {
        return 1.0;
    }

    @Override
    public boolean shouldSimulatePID()
    {
        return true;
    }

    @Override
    public void act(float delta)
    {
        // double currHeight = this.prevHeight;
        // double currVelocity = this.prevVelocity;

        // double motorPower = 0.0;
        // FauxbotActuatorBase actuator = FauxbotActuatorManager.get(RobotSimulator.MotorChannel);
        // if (actuator != null && actuator instanceof FauxbotMotorBase)
        // {
        //     FauxbotMotorBase motor = (FauxbotMotorBase)actuator;
        //     motorPower = (float)motor.get();
        // }

        // currVelocity += motorPower * RobotSimulator.MotorStrength * delta + RobotSimulator.Gravity * delta;
        // currHeight += (currVelocity * delta);

        // if (currVelocity > RobotSimulator.ElevatorMaxVelocity)
        // {
        //     currVelocity = RobotSimulator.ElevatorMaxVelocity;
        // }
        // else if (currVelocity < RobotSimulator.ElevatorMinVelocity)
        // {
        //     currVelocity = RobotSimulator.ElevatorMinVelocity;
        // }

        // if (this.prevHeight > RobotSimulator.ElevatorMaxHeight)
        // {
        //     currHeight = RobotSimulator.ElevatorMaxHeight;
        //     currVelocity = 0.0f;
        // }
        // else if (this.prevHeight < RobotSimulator.ElevatorMinHeight)
        // {
        //     currHeight = RobotSimulator.ElevatorMinHeight;
        //     currVelocity = 0.0f;
        // }

        // this.prevHeight = currHeight;
        // this.prevVelocity = currVelocity;

        // FauxbotSensorBase sensor = FauxbotSensorManager.get(RobotSimulator.EncoderAChannel);
        // if (sensor != null && sensor instanceof FauxbotEncoder)
        // {
        //     FauxbotEncoder encoder = (FauxbotEncoder)sensor;
        //     encoder.set((int)this.prevHeight);
        // }
    }

    /**
     * Draw a frame of animation based on the current state of the simulation.
     * Remember that (0, 0) is at the bottom left!
     */
    @Override
    public void draw(Batch batch, float parentAlpha)
    {
        super.draw(batch, parentAlpha);

        // float elevatorHeightRatio = (float)this.prevHeight / (RobotSimulator.ElevatorMaxHeight - RobotSimulator.ElevatorMinHeight);

        // float frameX = this.getX();
        // float frameY = this.getY();
        // float frameHeight = this.getHeight();
        // float frameWidth = this.getWidth();

        // ShapeDrawer drawer = new ShapeDrawer(batch, new TextureRegion(this.drawerTexture, 0, 0, 1, 1));

        // // cycle through the floors:
        // Color elevatorCarColor = Color.RED;
        // for (double ratio : RobotSimulator.FloorHeightPercentages)
        // {
        //     // if we are within a small allowance from the floor, change elevator color to Green
        //     if (Math.abs(elevatorHeightRatio - ratio) < RobotSimulator.PercentageAllowance)
        //     {
        //         elevatorCarColor = Color.GREEN;
        //     }

        //     // draw the floor:
        //     drawer.setColor(Color.BLACK);
        //     drawer.setDefaultLineWidth(1.0f);
        //     drawer.line(
        //         frameX + RobotSimulator.ElevatorCarWidth,
        //         frameY + (float)ratio * frameHeight,
        //         frameX + frameWidth,
        //         frameY + (float)ratio * frameHeight); 
        // }

        // // draw the elevator car:
        // drawer.setColor(elevatorCarColor);
        // drawer.setDefaultLineWidth(1.0f);
        // drawer.filledRectangle(
        //     frameX,
        //     frameY + elevatorHeightRatio * frameHeight,
        //     RobotSimulator.ElevatorCarWidth,
        //     RobotSimulator.ElevatorCarHeight);

        // // draw the elevator rider:
        // batch.draw(
        //     this.elevatorPassenger,
        //     frameX,
        //     frameY + elevatorHeightRatio * frameHeight,
        //     RobotSimulator.ElevatorCarWidth,
        //     RobotSimulator.ElevatorCarHeight);
    }

    @Override
    public void dispose()
    {
        super.dispose();
        // this.drawerTexture.dispose();
        // this.elevatorPassenger.dispose();
    }
}
