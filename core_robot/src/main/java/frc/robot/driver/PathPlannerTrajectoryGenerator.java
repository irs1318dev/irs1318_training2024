package frc.robot.driver;

import frc.lib.driver.TrajectoryManager;
import frc.lib.robotprovider.IPathPlanner;
import frc.lib.robotprovider.ITrajectory;
import frc.lib.robotprovider.PathPlannerRotationTarget;
import frc.lib.robotprovider.PathPlannerWaypoint;
import frc.lib.robotprovider.Point2d;
import frc.robot.AutonLocManager;
import frc.robot.TuningConstants;

public class PathPlannerTrajectoryGenerator
{
    public static void generateTrajectories(TrajectoryManager trajectoryManager, IPathPlanner pathPlanner)
    {
        PathPlannerTrajectoryGenerator.generateTrajectories(false, trajectoryManager, pathPlanner);
        PathPlannerTrajectoryGenerator.generateTrajectories(true, trajectoryManager, pathPlanner);

        // ------------------------------- Macro paths --------------------------------------------
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.REVDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 180.0, 0.0),
                new PathPlannerWaypoint(-15.0, 0.0, 180.0, 0.0),
                new PathPlannerWaypoint(-30.0, 0.0, 180.0, 0.0)),

            "goBackwards30in");
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.REVDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 180.0, 0.0),
                new PathPlannerWaypoint(-12.0, 0.0, 180.0, 0.0)),
            "goBackwards1ft");
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.REVDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 180.0, 0.0),
                new PathPlannerWaypoint(-6.0, 0.0, 180.0, 0.0),
                new PathPlannerWaypoint(-12.0, 0.0, 180.0, 0.0)),
            "goBackwards15in");
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.REVDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.REVDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 90.0, 0.0),
                new PathPlannerWaypoint(9.0, 16.0, 0.0, 0.0),
                new PathPlannerWaypoint(18.0, 32.0, 0.0, 0.0)),
            "goLeft32inForward18in");
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.REVDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.REVDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 270.0, 0.0),
                new PathPlannerWaypoint(9.0, -16.0, 0.0, 0.0),
                new PathPlannerWaypoint(18.0, -32.0, 0.0, 0.0)),
            "goRight32inForward18in");
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.REVDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.REVDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 90.0, 0.0),
                new PathPlannerWaypoint(0.0, 11.0, 90.0, 0.0),
                new PathPlannerWaypoint(0.0, 22.0, 90.0, 0.0)),
            "goLeft22in");
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.REVDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.REVDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 270.0, 0.0),
                new PathPlannerWaypoint(0.0, -22.0, 270.0, 0.0)),
            "goRight22in");

        // ------------------------------- Auton Paths --------------------------------------------
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
                //new PathPlannerWaypoint(0.0, 0.0, 0.0, 0.0)
                ),
            "goJamieTask");
            
        
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.REVDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                

//ORIGINAL: RESULT: 180 turn on the way while veering left, then corrects itself (goes right towards) endpoint
                new PathPlannerWaypoint(0.0, 0.0,180, 180),
                new PathPlannerWaypoint(60.0, 0.0, 180, 180.0),
                new PathPlannerWaypoint(120.0, 0.0, 180, 180.0)
                

// EXPERIMENT 1: RESULT: SAME AS ORIGINAL
/* 
                new PathPlannerWaypoint(0.0, 0.0,0, 180),
                new PathPlannerWaypoint(60.0, 0.0, 0, 180),
                new PathPlannerWaypoint(120.0, 0.0, 0, 180)
*/

// EXPERIMENT 2: RESULT: CRASH AND THEN SUCCESS
                
                /*new PathPlannerWaypoint(0.0, 0.0,0, 0),
                new PathPlannerWaypoint(30.0, 0.0,0, 0),
                new PathPlannerWaypoint(60.0, 0.0, 0, 0)*/
                


                 //,
                
                //new PathPlannerWaypoint(0.0, 0.0, 0.0, 180.0)
                ),
            "goForwards5ft");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.REVDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.REVDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                

                
                new PathPlannerWaypoint(60, 0.0,180.0, 0),
                new PathPlannerWaypoint(30.0, 0.0, 180.0, 0),
                new PathPlannerWaypoint(0.0, 0.0, 180.0, 0)
                
                
                ),
            "goBackwards5ft");

        
    
    }

    public static void generateTrajectories(boolean isRed, TrajectoryManager trajectoryManager, IPathPlanner pathPlanner)
    {
        double framePreremetere = 22.5; //With bumpers
        double halfFramePreremetere = framePreremetere / 2.0;  

        Point2d P1 = new Point2d(getXPosition(isRed, 326.5 - halfFramePreremetere), 64 + halfFramePreremetere);
        Point2d P2 = new Point2d(getXPosition(isRed, 289 - 21.22) , 198); //210 degrees orientation
        Point2d P3 = new Point2d(getXPosition(isRed, 250.5 + halfFramePreremetere), 306 - halfFramePreremetere);

        Point2d P4 = new Point2d(getXPosition(isRed, 288 - halfFramePreremetere), 239 - halfFramePreremetere);

        Point2d P5 = new Point2d(getXPosition(isRed, 212), 162);
        Point2d P5M = new Point2d(getXPosition(isRed, P5.x + 20), P5.y);

        Point2d P6 = new Point2d(getXPosition(isRed, 212), 219);
        Point2d P6M = new Point2d(getXPosition(isRed, P6.x + 20), P6.y);

        Point2d P7 = new Point2d(getXPosition(isRed, 212), 276);
        Point2d P7M = new Point2d(getXPosition(isRed, P7.x + 20), P7.y);

        Point2d P8 = new Point2d(getXPosition(isRed, 0), 29.64);
        Point2d P8M = new Point2d(getXPosition(isRed, P8.x + 20), P8.y);

        Point2d P9 = new Point2d(getXPosition(isRed, 0), 95.64);
        Point2d P9M = new Point2d(getXPosition(isRed, P9.x + 20), P9.y);

        Point2d P10 = new Point2d(getXPosition(isRed, 0), 161.64);
        Point2d P10M = new Point2d(getXPosition(isRed, P10.x + 20), P10.y);
        
        Point2d P11 = new Point2d(getXPosition(isRed, 0), 227.64);
        Point2d P11M = new Point2d(getXPosition(isRed, P11.x + 20), P11.y);

        Point2d P12 = new Point2d(getXPosition(isRed, 0), 293.64);
        Point2d P12M = new Point2d(getXPosition(isRed, P12.x + 20), P12.y);

        //ToDO fix 13 14
        Point2d P13 = new Point2d(getXPosition(isRed, 0), 93.154754 - halfFramePreremetere);
        Point2d P14 = new Point2d(getXPosition(isRed, 0), 231.777 + halfFramePreremetere);
        //ToDo : add p15, p16, p17
        Point2d P15 = new Point2d(0,0);//Might use


        Point2d P16 = new Point2d(getXPosition(isRed, 172.955),  93.154754 - halfFramePreremetere);
        Point2d P17 = new Point2d(getXPosition(isRed, 172.955),  231.777 + halfFramePreremetere);

        Point2d P18 = new Point2d(getXPosition(isRed, 115 + 21 + halfFramePreremetere), 162);

        Point2d P19 = new Point2d(getXPosition(isRed, 204), 107);
        Point2d P20 = new Point2d(getXPosition(isRed, 204), 217);

        Point2d P21 = new Point2d(getXPosition(isRed, 72), 0);

        //Verified Red
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P3, getOrientationOrHeading(isRed, 225), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P7M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P7, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P6M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P6, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P5M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P5, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180))),
                isRed?"P3toP5Red":"P3toP5Blue");

        
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P2, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 210)), // fix orientation
                new PathPlannerWaypoint(P5M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P5, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P5M, getOrientationOrHeading(isRed, 255), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P16, getOrientationOrHeading(isRed, 225), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P8M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P8, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180))),
                "P2toP8");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P4, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P6M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P6, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P18, getOrientationOrHeading(isRed, 225), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P10M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P10, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P11M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P11, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180))),
                isRed ? "P4toP11Red" : "P4toP11Blue");


        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P1, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P8M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P8, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P9M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P9, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P10M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P10, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P11M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P11, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P12M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P12, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180))),
            isRed ? "P1toP12Red" : "P1toP12Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P3, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P7M, getOrientationOrHeading(isRed, 225), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P7, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                //Stop for Note Pickup
                new PathPlannerWaypoint(P12M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P12, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                //Possible Note Pickup
                new PathPlannerWaypoint(P11M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P11, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                //Possible Note Pickup
                new PathPlannerWaypoint(P10M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P10, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                //Possible Note Pickup
                new PathPlannerWaypoint(P9M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P9, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                //Possible Note Pickup
                new PathPlannerWaypoint(P8M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P8, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180))),
                //Possible Note Pickup
                isRed? "P3toP8Red" : "P3toP8Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P2, getOrientationOrHeading(isRed, 135), getOrientationOrHeading(isRed, 210)),
                new PathPlannerWaypoint(P6M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P6, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P14, getOrientationOrHeading(isRed, 135), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P10M, getOrientationOrHeading(isRed, 225), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P10, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180))),
                isRed? "P2toP10Red" : "P2toP10Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(getXPosition(isRed, 0),0, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(getXPosition(isRed, 0),80, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(getXPosition(isRed,-80),80, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(getXPosition(isRed,-80),0, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(getXPosition(isRed,0),0, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180))),
                isRed? "P3toP16Red" : "P3toP16Blue"); 
                
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P3, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 0)),
                new PathPlannerWaypoint(P17, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 0))
                //new PathPlannerWaypoint(getXPosition(isRed,-80),80, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 0)),
                //new PathPlannerWaypoint(getXPosition(isRed,-80),0, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 0)),
                //new PathPlannerWaypoint(getXPosition(isRed,0),0, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 0))
                ),
                isRed? "AyushTestRed" : "AyushTestBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(

            
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,

                /* 
                new PathPlannerWaypoint(P3, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)), 
                new PathPlannerWaypoint(P20, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P6M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P4, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 0))//end facing left
*/

                new PathPlannerWaypoint(P12, getOrientationOrHeading(isRed, 270), getOrientationOrHeading(isRed, 180)), 
                new PathPlannerWaypoint(P11, getOrientationOrHeading(isRed, 270), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P10, getOrientationOrHeading(isRed, 270), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P9, getOrientationOrHeading(isRed, 270), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P8, getOrientationOrHeading(isRed, 270), getOrientationOrHeading(isRed, 180))
                //new PathPlannerWaypoint(P4, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 0))//end facing left

                
                //new PathPlannerWaypoint(P1, getOrientationOrHeading(isRed, 225), getOrientationOrHeading(isRed, 0))//end facing us
                //new PathPlannerWaypoint(getXPosition(isRed,-80),80, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 0)),
                //new PathPlannerWaypoint(getXPosition(isRed,-80),0, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 0)),
                //new PathPlannerWaypoint(getXPosition(isRed,0),0, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 0))

                //new PathPlannerWaypoint(P3, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)), 
                //new PathPlannerWaypoint(P6M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 90)),
                //new PathPlannerWaypoint(P7, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 270))
                ),
                isRed? "JamieAndAyushPathRed" : "JamieAndAyushPathBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P4, getOrientationOrHeading(isRed, 225), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P5, getOrientationOrHeading(isRed, 90), getOrientationOrHeading(isRed, 90)),
                new PathPlannerWaypoint(P6, getOrientationOrHeading(isRed, 90), getOrientationOrHeading(isRed, 90)),
                new PathPlannerWaypoint(P7, getOrientationOrHeading(isRed, 90), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P12, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed,180)),
                new PathPlannerWaypoint(P20, getOrientationOrHeading(isRed, 135), getOrientationOrHeading(isRed,180)),
                new PathPlannerWaypoint(P14, getOrientationOrHeading(isRed, 135), getOrientationOrHeading(isRed,180)),
                new PathPlannerWaypoint(P11, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed,180)),
                new PathPlannerWaypoint(P14, getOrientationOrHeading(isRed, 225), getOrientationOrHeading(isRed,180)),
                new PathPlannerWaypoint(P10, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed,180)),
                new PathPlannerWaypoint(P18, getOrientationOrHeading(isRed, 45), getOrientationOrHeading(isRed,180)),
                new PathPlannerWaypoint(P20, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed,180))

                //new PathPlannerWaypoint(getXPosition(isRed,-80),80, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 0)),
                //new PathPlannerWaypoint(getXPosition(isRed,-80),0, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 0)),
                //new PathPlannerWaypoint(getXPosition(isRed,0),0, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 0))
                ),
                isRed? "SevenNoteAutoRed" : "SevenNoteAutoBlue");
    }


    public static double getXPosition(boolean isRed, double position)
    {
        if(isRed)
        {
            return position;
        }
        else
        {
            return position * -1.0;
        }
    }

    //TODO can getOrientationorHeading() go in AutonLocManager?

    public static double getOrientationOrHeading(boolean isRed, double orientationOrHeading)
    {
        if(isRed)
        {
            return orientationOrHeading;
        }
        else
        {
            return 180.0 - orientationOrHeading;
        }
    }

    private static void addTrajectory(TrajectoryManager trajectoryManager, ITrajectory trajectory, String name)
    {
        // ExceptionHelpers.Assert(trajectory != null, "Adding null trajectory '%s'!", name);
        try
        {
            trajectoryManager.addTrajectory(name, trajectory);
        }
        catch (Exception ex)
        {
            System.err.println("Encountered exception generating path " + name + ": " + ex.toString());
            if (TuningConstants.THROW_EXCEPTIONS)
            {
                throw ex;
            }
        }
    }
}