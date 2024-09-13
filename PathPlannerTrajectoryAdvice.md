# Guide to building your own Autonomous Path

## Overview

Writing your own path in PathPlanner is immensely simple. The process of adding a trajectory to PathPlannerTrajectory involves using a function of the same name: `addTrajectory()`. The `addTrajectory` function takes three parameters: a TrajectoryManager, an ITrajectory, and a string for a name.

## Making the Path

### TrajectoryManager

For this part you only need to write `trajectoryManager`. The details of what it is and how it works are effectively irrelevant for your purposes. All that is necessary to be done for this part is to pass in `trajectoryManager` as the first parameter. Note that all parameters in Java are postional arguments, and no analogue to keyword arguments exists.

### ITrajectory

Similar to the previous part, comprehension of what an `ITrajectory` is irrelevant to your utilization of the PathPlanner's Functionality. The `ITrajectory` is itself passed in by running another function. The function is `pathPlanner.buildTrajectory()`.

The definition of pathPlanner.buildTrajectory() is as follows: 

```java 
    public ITrajectory buildTrajectory(
        double maxVelocity,
        double maxAcceleration,
        double maxAngularVelocity,
        double maxAngularAcceleration,
        IPathPlannerGoal... goalPoints);
```

Here, all arguments necessary to pass in can be readily idenrtified based on the name of the parameters as in the definition in the aforementioned code block. Generally, you can pass in the following for the first four parameters:

```java
    TuningConstants.REVDRIVETRAIN_LOW_PATH_TRANSLATIONAL_VELOCITY,
    TuningConstants.REVDRIVETRAIN_LOW_PATH_TRANSLATIONAL_ACCELERATION,
    TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
    TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
```

This is due to the fact that these variables are generally tied to the drivetrain and its related facilities.

For the final parameter, notice the `...` after the `IPathPlannerGoal`. If you have not encountered this symbol before, it means that the associated parameter is variadic. A variadic parameter means that, as the name suggests, a variable number of inputs of the required type may be passed in to the function. The parameter `goalPoints` thus represents all the points along your path.

To see an example of what variadic means in practice , look at the following code blocks extracted from our repository.

Block 1, Cleaned version of the "goJamieTask" path:
```java
    addTrajectory(
        trajectoryManager,
        pathPlanner.buildTrajectory(
            TuningConstants.REVDRIVETRAIN_LOW_PATH_TRANSLATIONAL_VELOCITY,
            TuningConstants.REVDRIVETRAIN_LOW_PATH_TRANSLATIONAL_ACCELERATION,
            TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
            TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,

            new PathPlannerWaypoint(0.0, 0.0, 0.0, 0.0),
            new PathPlannerWaypoint(0.0, 80.0, 0.0, 180.0),
            new PathPlannerWaypoint(0.0, -0.0, 0.0, 0.0),
            new PathPlannerWaypoint(0.0, 0.0, 0.0, 0.0)
            ),
        "goJamieTask");
```

Block 2, Cleaned version of "goForwards5ft" path:
```java
    addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.REVDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
    
                new PathPlannerWaypoint(0.0, 0.0,180, 180),
                new PathPlannerWaypoint(60.0, 0.0, 180, 180.0),
                new PathPlannerWaypoint(120.0, 0.0, 180, 180.0)
                ),
            "goForwards5ft");
```

As you notice, the former method has 4 `PathPlannerWaypoint` points, whereas the latter has only 3. Ultimately, the goal is for you to understand the notion of variadic parameters and how we use them to make our autonomous paths.

#### PathPlannerWaypoint: The only part where you have to use your head

The parameters that were passed in to the final argument were of type `PathPlannerWaypoint` and not `IPathPlannerGoal`. This does not matter. What matters is understanding what a `PathPlannerWaypoint` does and how to use it.

The whole point (ironic but unintentional) of a `PathPlannerWaypoint` is to denote a physical position of the robot through 4 numbers. The defintion of the `PathPlannerWaypoint` constructor is as follows:

```java
    /**
     * Creates a waypoint at position (x, y), traveling in the direction of the heading, facing the orientation, with an overridden velocity
     * @param x position (in inches)
     * @param y position (in inches)
     * @param heading travel direction (tangent, in degrees)
     * @param orientation facing direction (in degrees)
     */
    public PathPlannerWaypoint(double x, double y, double heading, double orientation)
    {
        this(x, y, heading, OptionalDouble.of(orientation));
    }
```

Thus, to define a point we take its `x` and `y` coordinates, its heading, and its orientation. The `heading` and `orientation` are just fancy words to tell us about which way the robot will move and which the robot will face, respectively.

Once you have extrapolated these values, you are set for (hopefully) success.

### Name

The name is what the task will be known as and how it will be referenced when employing the path in other files. The name is 
a Java `String` and can be anything that you please.

## Executing the path

The path that you have made will be deployed through the use of a `MacroOperationDescription`. Below is an example of a generic Macro that will run the path called `goLeft32inForward18in`.

```java
new MacroOperationDescription(
    MacroOperation.FollowPathTest1,
    UserInputDevice.Test1,
    0,
    EnumSet.of(Shift.Test1Debug),
    EnumSet.noneOf(Shift.class),
    ButtonType.Toggle,
    () -> SequentialTask.Sequence(
        new FollowPathTask("goLeft32inForward18in", Type.RobotRelativeFromCurrentPose)
    ),
    //rest is irrelevant for your purposes
)
```

The `ONLY` part you need to modify is the code inside the `FollowPathTask`. The `String` passed as the first argument should be the name of the path you have designed. The second argument represents whether the coordinates are `field relative`, `robot relative`, or `absolute` in nature.