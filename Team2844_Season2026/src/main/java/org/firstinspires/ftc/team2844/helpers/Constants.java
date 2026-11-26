package org.firstinspires.ftc.team2844.helpers;


import com.pedropathing.math.Pose;

import org.firstinspires.ftc.team2844global.collision.CollisionBox;
import org.firstinspires.ftc.team2844global.PIDF;
import org.firstinspires.ftc.team2844global.GlobalConstants;

public class Constants {

    /* ---------- VELOCITY PIDS ---------- */
    //TODO PUT REAL PIDS IN
    public static final double NP = 11.3;
    public static final double NI = 0.0;
    public static final double ND = 0.0;
    public static final double NS = 0.0;
    public static final double NV = 1.5;
    public static final PIDF NECTAR_PIDF = new PIDF(NP, NI, ND, NS, NV);


    public static final double PP = 11.3;
    public static final double PI = 0.0;
    public static final double PD = 0.0;
    public static final double PS = 0.0;
    public static final double PV = 1.5;
    public static final PIDF POLLEN_PIDF = new PIDF(PP, PI, PD, PS, PV);


    /* ---------- TURRET CONSTANTS ---------- */
    public static final double MAX_ROTATION = 152.96337;
    public static final double MIN_ROTATION = -180.0;
    public static final double MAX_SERVO_RANGE = 350.0;
    public static double TURRET_THRESHOLD_DEGREES = 1.0;
    public static final double MIN_ENCODER = 0.18;
    public static final double MAX_ENCODER = 3.0;

    private static final double SERVO_TO_GEAR = 2.0;
    private static final double GEAR_TO_TURRET = 176.0/90.0;
    private static final double TOTAL_RATIO = GEAR_TO_TURRET/SERVO_TO_GEAR;
    /**Divide the servo degrees by this number to get the turret degrees.
     * Or multiply turret degrees by this number to get the servo degrees
     */
    //public static final double TURRET_DEGREES_TO_SERVO_DEGREES = TOTAL_RATIO;

    /**Divide the turret degrees by this number to get the encoder degrees.
     * Or multiply encoder degrees by this number to get turret degrees
     */
    //public static final double TURRET_TO_ENCODER = 40.0/176.0;

    /* ---------- SHOOTER CONSTANTS ---------- */
    /**MEASURED IN RPM, MUST BE CONVERTED TO TICKS PER SECOND FOR THE SHOOTER CMD*/
    public static double SHOOTER_IDLE = 800;
    public static double SHOOT_RPM = 4000;
    public static double HOOD_DEFAULT = 0.0625;
    public static final double SHOOTER_RADIUS = 36.0/25.4;


    /* ---------- IRL ROBOT SIZES ---------- */
    //TODO PUT REAL WIDTH, LENGTH, AND HEIGHT
    public static final double ROBOT_WIDTH = 14.0;
    public static final double ROBOT_LENGTH = 18.0;
    public static final double ROBOT_HEIGHT = 18.0;
    public static final double LL_HEIGHT = 15.0; //TODO get real height


    /* ---------- Set Speeds and Deadbands ---------- */

    // ?----- intake and transfer -----
    public static double INTAKE_SPEED = 1.0;
    public static double TRANSFER_SPEED = 0.6;
    public static double FAST_TRANSFER = 1.0;

    public static  double OPEN_GATE = 0.13;
    public static  double CLOSED_GATE = 0.0;
    //TODO, REAL EMPTY TIME
    public static final long EMPTY_TIME = 500;

    public static final double FLOWER_DOWN = 0.0;
    public static final double FLOWER_UP = 0.84;

    // ?----- controller -----
    public static double STICK_DEADBAND = 0.05;

    // ?----- shooter and hood -----
    public static double VEL_RPM_THRESHOLD = 200;
    public static double MAX_HOOD = 0.15;
    public static double MIN_HOOD = 0.0;
    public static double MAX_HOOD_DEGREE = 90.0;
    public static double MIN_HOOD_DEGREE = 60.0;
    public static double EFFICIENCY = 0.85;



    /* ---------- PEDRO PATHING ---------- */
    public static final double XPOD_OFFSET = -1.87;
    public static final double YPOD_OFFSET = -7.0;

    /* ---------- AUTO DRIVING ---------- */
    public static final double SAFETY_MARGIN = 1.5;
    public static final double MAX_CURVE_DIS = 18.0;
    /**Extra room added to the safe corners so lines between them don't count as touching.*/
    public static final double WAYPOINT_EPSILON = 0.5;
    /**How far the safe corners are pushed out from each obstacle: the reach of the robot box
     * (with safety margin) from its center at any heading, plus WAYPOINT_EPSILON.*/
    public static final double SAFE_OFFSET = Math.hypot(
            ROBOT_LENGTH / 2.0 + SAFETY_MARGIN,
            ROBOT_WIDTH / 2.0 + SAFETY_MARGIN) + WAYPOINT_EPSILON;

    /**
     * Waypoints for driving around each obstacle. SAFE_COLLISIONS[j] goes with GlobalConstants.COLLISIONS[j].
     * <p>Each corner is pushed SAFE_OFFSET out in both x and y. Corners the robot can't reach
     * (closer than SAFE_OFFSET to a wall) are left out, so a flower against a wall only has 2.
     * <p>Only use these as places to aim for. Collision checks should still use GlobalConstants.COLLISIONS.
     */
    public static final CollisionBox[] SAFE_COLLISIONS = makeSafeCollisions(GlobalConstants.COLLISIONS);

    /**Builds SAFE_COLLISIONS. Only works for axis-aligned boxes.*/
    private static CollisionBox[] makeSafeCollisions(CollisionBox[] boxes){
        CollisionBox[] safe = new CollisionBox[boxes.length];
        for(int j = 0; j < boxes.length; j++){
            Pose[] corners = boxes[j].getCorners();

            //center of the box, used to tell which way is "out" for each corner
            double centerX = 0.0;
            double centerY = 0.0;
            for(Pose c : corners){
                centerX += c.x();
                centerY += c.y();
            }
            centerX /= corners.length;
            centerY /= corners.length;

            java.util.List<Pose> safeCorners = new java.util.ArrayList<>();
            for(Pose c : corners){
                double x = c.x() + SAFE_OFFSET * Math.signum(c.x() - centerX);
                double y = c.y() + SAFE_OFFSET * Math.signum(c.y() - centerY);
                boolean reachable = x >= SAFE_OFFSET && x <= GlobalConstants.FIELD_SIZE - SAFE_OFFSET
                        && y >= SAFE_OFFSET && y <= GlobalConstants.FIELD_SIZE - SAFE_OFFSET;
                if(reachable){
                    safeCorners.add(new Pose(x, y));
                }
            }
            safe[j] = new CollisionBox(safeCorners.toArray(new Pose[0]));
        }
        return safe;
    }


    /* ---------- Hardware Map Names ---------- */
    //Control Hub
    public static final String CHM0 = "leftFront";
    public static final String CHM1 = "leftBack";
    public static final String CHM2 = "rightBack";
    public static final String CHM3 = "rightFront";


    public static final String CHS0 = "nectarHood";
    public static final String CHS1 = "pollenHood";
//    public static final String CHS2 = "gate";
    public static final String CHS2 = "nectarGate";
    public static final String CHS3 = "pollenGate";
    public static final String CHS4 = "axon1";
    public static final String CHS5 = "axon2";

    public static final String CHI2C0 = "";
    public static final String CHI2C1 = "pinpoint";
    public static final String CHI2C2 = "huskyLens";
    public static final String CHI2C3 = "";

    public static final String CHAI0 = "";
    public static final String CHAI1 = "";
    public static final String CHAI2 = "";
    public static final String CHAI3 = "";

    public static final String CHDD0 = "";
    public static final String CHDD1 = "";
    public static final String CHDD2 = "";
    public static final String CHDD3 = "";
    public static final String CHDD4 = "";
    public static final String CHDD5 = "";
    public static final String CHDD6 = "";
    public static final String CHDD7 = "";


    public static final String CHLL = "limelight";

    // Expansion Hub
    public static final String EHM0 = "intake";
    public static final String EHM1 = "transfer";
    public static final String EHM2 = "nectarShooter";
    public static final String EHM3 = "pollenShooter";


    public static final String EHS0 = "flowerRamp";
    public static final String EHS1 = "";
    public static final String EHS2 = "";
    public static final String EHS3 = "light0";
    public static final String EHS4 = "light1";
    public static final String EHS5 = "light2";

    public static final String EHI2C0 = "";
    public static final String EHI2C1 = "";
    public static final String EHI2C2 = "";
    public static final String EHI2C3 = "";

    public static final String EHAI0 = "encoder1";
    public static final String EHAI1 = "encoder2";
    public static final String EHAI2 = "absolute";
    public static final String EHAI3 = "";

    public static final String EHDD0 = "n0";
    public static final String EHDD1 = "n1";
    public static final String EHDD2 = "n2";
    public static final String EHDD3 = "n3";
    public static final String EHDD4 = "p0";
    public static final String EHDD5 = "p1";
    public static final String EHDD6 = "p2";
    public static final String EHDD7 = "p3";


    //CLOCKWISE rotation is negative heading (approaching -180)
    //COUNTERCLOCKWISE rotation is positive heading (approaching 180)
}
