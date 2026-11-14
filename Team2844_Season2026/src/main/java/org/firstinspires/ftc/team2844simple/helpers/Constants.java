package org.firstinspires.ftc.team2844simple.helpers;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

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

    /* ---------- MATH CONSTANTS ---------- */
    /** Multiply to turn radians per second into rotations per minute, divide to get the opposite*/
    public static final double RAD_PS_TO_RPM = 30.0 / Math.PI;
    /** Multiply to turn meters to inches, and divide to get inches to meters*/
    public static final double METERS_TO_INCHES = 39.3701;

    /* ---------- MISSING INFO ---------- */
    public static final double NO_HEADING = -999;
    public static final double NO_POS = -999;
    public static final double NO_DIS = -999;
    public static final double NO_LL = -999;
    public static final int NO_TARGET = -1;
    public static final int NO_ALLIANCE = -1;
    public static final double TOO_SLOW = -999;
    public static final double TOO_FAST = 999;


    /* ---------- COORDINATE DATA ---------- */
    public static double targetX = Constants.NO_POS;
    public static double targetY = Constants.NO_POS;

    public static final double CELL_HEIGHT = 53.5;

    public static final double BLUE_AUDIENCE_X = 84.0;
    public static final double BLUE_AUDIENCE_Y = 66.0;
    public static final double BLUE_BACK_X = 84.0;
    public static final double BLUE_BACK_Y = 78.0;
    public static final double RED_AUDIENCE_X = 60.0;
    public static final double RED_AUDIENCE_Y = 66.0;
    public static final double RED_BACK_X = 60.0;
    public static final double RED_BACK_Y = 78.0;


    /* ---------- ID CONSTANTS ---------- */
    public static final int BLUE_AUDIENCE = 0;
    public static final int BLUE_BACK = 1;
    public static final int RED_AUDIENCE = 2;
    public static final int RED_BACK = 3;

    // ?----- alliance tracker -----
    public static int alliance = NO_ALLIANCE;
    public static final int BLUE_ALLIANCE = 0;
    public static final int RED_ALLIANCE = 2;
    public static void changeAlliance(int team){
        alliance = team;
    }

    // ?----- side ids -----
    public static final int AUDIENCE_SIDE = 0;
    public static final int BACK_SIDE = 1;
    public static int side = NO_TARGET;

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
    public static double VEL_RPM_THRESHOLD = 200;
    /**MEASURED IN RPM, MUST BE CONVERTED TO TICKS PER SECOND FOR THE SHOOTER CMD*/
    public static double SHOOTER_IDLE = 800;
    public static double SHOOT_RPM = 3200;
    public static double EFFICIENCY = 0.85;
    public static final double SHOOTER_RADIUS = 36.0/25.4;

    public static void changeSide(int newSide){
        side = newSide;
        setTarget();
    }

    public static void setTarget(){
        changeTarget(side+alliance);
    }

    public static int target = NO_TARGET;

    /**
     * This method changes the values for the goal's field coordinates depending on case given
     * The field coordinates are rough estimates of where the cell is above when tipped up.
     * @param t 0- Blue Audience 1- Blue Back 2- Red Audience 3- Red Back
     *
     */
    public static void changeTarget(int t) {
        target = t;
        switch(target) {
            case(Constants.BLUE_AUDIENCE): //Blue Audience
                targetX = Constants.BLUE_AUDIENCE_X;
                targetY = Constants.BLUE_AUDIENCE_Y;
                break;
            case(Constants.BLUE_BACK): //Blue Back
                targetX = Constants.BLUE_BACK_X;
                targetY = Constants.BLUE_BACK_Y;
                break;
            case(Constants.RED_AUDIENCE): //Red Audience
                targetX = Constants.RED_AUDIENCE_X;
                targetY = Constants.RED_AUDIENCE_Y;
                break;
            case(Constants.RED_BACK): //Red Back
                targetX = Constants.RED_BACK_X;
                targetY = Constants.RED_BACK_Y;
                break;
            default:
                System.out.println("Target Failed to Change");
                break;
        }
    }


    /* ---------- PEDRO PATHING ---------- */
    public static final double XPOD_OFFSET = -0.4321152018749808;
    public static final double YPOD_OFFSET = 0.7009387579489882;


    /**
     * This is a Config class for the turrets so that all the values can be stored in one parameter.
     * */
    public static class ShooterConfig{
        public final double kP, kI, kD, kS, kV;
        public final String motorName;
        public ShooterConfig(String motorName, double kP, double kI, double kD, double kS, double kV){
            this.motorName = motorName;
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kS = kS;
            this.kV = kV;
        }

        public ShooterConfig(String motorName, PIDF pidf){
            this.motorName = motorName;
            this.kP = pidf.kP;
            this.kI = pidf.kI;
            this.kD = pidf.kD;
            this.kS = pidf.kS;
            this.kV = pidf.kV;
        }
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

    /* COLORS */
    public static final double OFF = 0.00;
    public static final double RED = 0.28;
    public static final double ORANGE = 0.33;
    public static final double YELLOW = 0.39;
    public static final double SAGE = 0.44;
    public static final double GREEN = 0.50;
    public static final double AZURE = 0.56;
    public static final double BLUE = 0.61;
    public static final double INDIGO = 0.67;
    public static final double VIOLET = 0.72;
    public static final double WHITE = 1.00;


    //CLOCKWISE rotation is negative heading (approaching -180)
    //COUNTERCLOCKWISE rotation is positive heading (approaching 180)
}
