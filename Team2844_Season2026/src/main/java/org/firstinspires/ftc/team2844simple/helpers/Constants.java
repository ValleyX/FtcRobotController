package org.firstinspires.ftc.team2844simple.helpers;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.team2844global.collision.CollisionBox;
import org.firstinspires.ftc.team2844global.PIDF;
import org.firstinspires.ftc.team2844global.GlobalConstants;

@Config
@Configurable
public class Constants {

    /* ---------- VELOCITY PIDS ---------- */
    //TODO PUT REAL PIDS IN
    public static final double PP = 7.5;
    public static final double PI = 0.0;
    public static final double PD = 0.0;
    public static final double PS = 0.0;
    public static final double PV = 1.0;
    public static final PIDF POLLEN_PIDF = new PIDF(PP, PI, PD, PS, PV);

    /* ---------- IRL ROBOT SIZES ---------- */
    //TODO PUT REAL WIDTH, LENGTH, AND HEIGHT
    public static final double ROBOT_WIDTH = 15.875;
    public static final double ROBOT_LENGTH = 15.875;
    public static final double ROBOT_HEIGHT = 13.0;
    public static final double LL_HEIGHT = 15.0; //TODO get real height


    /* ---------- Set Speeds and Deadbands ---------- */

    // ?----- intake and transfer -----
    public static double INTAKE_SPEED = 1.0;
    public static double TRANSFER_SPEED = 0.6;
    public static double FAST_TRANSFER = 1.0;

    public static  double OPEN_GATE = 0.80;
    public static  double CLOSED_GATE = 0.5;
    public static final long EMPTY_TIME = 500;

    // ?----- controller -----
    public static double STICK_DEADBAND = 0.05;

    // ?----- turn to target while driving (TurnToTargetCmd) -----
    //TODO TUNE ON THE ROBOT, these are starting guesses
    /**Turn power per radian of heading error*/
    public static double AIM_KP = 1.0;
    /**Turn power per radian per second of change in heading error (damping)*/
    public static double AIM_KD = 0.05;
    /**Minimum turn power to get the robot rotating, only added when outside AIM_TOLERANCE_DEG*/
    public static double AIM_KS = 0.05;
    /**Largest turn power the aim is allowed to command, 0 to 1*/
    public static double AIM_MAX_TURN = 0.8;
    /**Heading error in degrees that counts as aimed*/
    public static double AIM_TOLERANCE_DEG = 1.0;

    // ?----- shooter -----
    public static double VEL_RPM_THRESHOLD = 150;
    /**MEASURED IN RPM, MUST BE CONVERTED TO TICKS PER SECOND FOR THE SHOOTER CMD*/
    public static double SHOOTER_IDLE = 800;
    public static double SHOOT_RPM = 3225;
    public static double EFFICIENCY = 0.85;
    public static final double SHOOTER_RADIUS = 36.0/25.4;


    /* ---------- PEDRO PATHING ---------- */
    public static final double XPOD_OFFSET = -0.4321152018749808;
    public static final double YPOD_OFFSET = 0.7009387579489882;

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


    public static final String CHS0 = "gate1";
    public static final String CHS1 = "";
//    public static final String CHS2 = "gate";
    public static final String CHS2 = "";
    public static final String CHS3 = "";
    public static final String CHS4 = "";
    public static final String CHS5 = "";

    public static final String CHI2C0 = "";
    public static final String CHI2C1 = "pinpoint";
    public static final String CHI2C2 = "";
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
    public static final String EHM2 = "";
    public static final String EHM3 = "pollenShooter";


    public static final String EHS0 = "gate2";
    public static final String EHS1 = "";
    public static final String EHS2 = "";
    public static final String EHS3 = "";
    public static final String EHS4 = "";
    public static final String EHS5 = "";

    public static final String EHI2C0 = "";
    public static final String EHI2C1 = "";
    public static final String EHI2C2 = "";
    public static final String EHI2C3 = "";

    public static final String EHAI0 = "";
    public static final String EHAI1 = "";
    public static final String EHAI2 = "";
    public static final String EHAI3 = "";

    public static final String EHDD0 = "";
    public static final String EHDD1 = "";
    public static final String EHDD2 = "";
    public static final String EHDD3 = "";
    public static final String EHDD4 = "p0";
    public static final String EHDD5 = "p1";
    public static final String EHDD6 = "p2";
    public static final String EHDD7 = "p3";

    //CLOCKWISE rotation is negative heading (approaching -180)
    //COUNTERCLOCKWISE rotation is positive heading (approaching 180)
}
