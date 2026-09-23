package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

@Config
@Configurable
public class Constants {

    /* ---------------- TURRET ---------------- */
    //24 tooth servo head to 130 turret teeth
    private static final double SERVO_TEETH = 130.0;
    private static final double TURRET_TEETH = 130.0; //r u sure its 130?
    private static final double TURRET_GEAR_RATIO = TURRET_TEETH/SERVO_TEETH;
    private static final double DEGREES_IN_FULL_SERVO_TURN = (360.0/TURRET_GEAR_RATIO);
    public static final double SERVO_DEGREE_TO_TURRET_DEGREE = TURRET_GEAR_RATIO;
//    public static final double DEGREES = 1/((5.0*24.0*360.0)/130.0);

    public static final double TURRET_THRESHHOLD = 3.0;

    public static final double MAX_DEGREE = 260.0;
    public static final double MIN_DEGREE = 10.0;

    public static final double MAX_TURN = MAX_DEGREE * SERVO_DEGREE_TO_TURRET_DEGREE;
    public static final double MIN_TURN = MIN_DEGREE * SERVO_DEGREE_TO_TURRET_DEGREE;
    public static final double TURN_TICK = 1;

    public static final double TURRET_OFFSET = 90.0;
    public static final double TURRET_GAIN = 0.009;
    public static final double GAIN_THRESH = 20.0;
    public static final double NEUTRAL_TURRET = 180.0;
    public static double RADIUS_FROM_CENTER = 3.5;
    /**How many degrees off of the Neutral position the turret will go to track the target when not shooting */
    public static double MAX_NEUTRAL = 40.0;

    public static final double MIN_VOLTAGE = 0.22;
    public static final double MAX_VOLTAGE = 3.06;


    /* ---------------- SHOOTER ---------------- */
    public static final double VELOCITY_THRESHHOLD = 100;
    public static final double MIN_VELOCITY = 1000;

    /* ---------------- DRIVING ---------------- */
    public static double DRIVE_CORRECTION = 1.0;


    /* ---------------- KICKER ---------------- */
    public static double KICKDOWN = 0.28;
    public static double KICKDOWN_EXTRA = 0.40;
    public static double KICKDOWN_LESS = 0.27;
    public static double KICKDOWN_INTAKE = 0.43;
    public static double KICKUP = 0.15;


    /* ---------------- REGRESSION ---------------- */
    public static double REG_STEP = 20;

    public static double P_GAIN = 0.22;
    public static double I_GAIN = 0.0;
    public static double D_GAIN = 0.0;

    public static double VEL_KS = 0.0;
    public static double VEL_KV = 1.1;


    /* ---------------- SPINDEXER (OLD) ---------------- */
    public static double SLOT_ZERO = 0.055;
    public static double SLOT_ONE = 0.055; //0.2/3 //axon 0.45
    public static double SLOT_TWO = 0.055; //2*(0.2/3) //axon 0.835

    public static double SLOT_ZERO_LOOPED_ONE = 0.055;//0.2;
    public static double SLOT_ONE_LOOPED_ONE = 0.45;//4*(0.2/3);
    public static double SLOT_TWO_LOOPED_ONE = 0.835;//5*(0.2/3);

    public static double[] SLOT_ARRAY = {SLOT_ZERO, SLOT_ONE, SLOT_TWO};//, SLOT_ZERO_LOOPED_ONE, SLOT_ONE_LOOPED_ONE, SLOT_TWO_LOOPED_ONE};


    /* ---------------- SORTING (OLD) ---------------- */
    public static final int MIN_COLOR_SUM = 900;
    public static final int MIN_ALPHA = 450;

    public static final int PURPLE_ARTIFACT = 1;
    public static final int GREEN_ARTIFACT = 0;

    /** If all colors added are equal to this number, they are sortable with one green and two purple*/
    public static final int SORTABLE = PURPLE_ARTIFACT + PURPLE_ARTIFACT;
    public static final int UNKNOWN_COLOR = 999;

    public static final int PATTERN_PPG = 2110;
    public static final int PATTERN_PGP = 2101;
    public static final int PATTERN_GPP = 2011;

    /* ---------------- LIMELIGHT ---------------- */
    public static final int NO_LL = -999;

    public static final int BLUE_PIPELINE = 0;
    public static final int RED_PIPELINE = 1;

    public static final int BLUE_PIPELINE_MOTIF = 2;
    public static final int RED_PIPELINE_MOTIF = 3;

    public static final double BLUE_APRILTAG_X = -58.3727;
    public static final double BLUE_APRILTAG_Y = -55.6425;

    public static final double RED_APRILTAG_X = -58.3727;
    public static final double RED_APRILTAG_Y = 55.6425;


    //Pinpoint
    public static final double NO_PP = -999.0;
    public static final double NO_HEADING = -999.0;

    //Good numbers to have
    public static final double METER_TO_INCH = 39.3701;



    /* ---------------- INTAKE AND TRANSFER ---------------- */
    public static final double INTAKE_SPEED = 1.0;

    public static final double SLOW_TFEED = 0.1;
    public static final double TFEED_SPEED = 1.0;


    /* ---------------- LIGHTS ---------------- */
    public static final int TOPL_INDEX = 0;
    public static final int MIDL_INDEX = 1;
    public static final int BOTL_INDEX = 2;
    public static final int LEFTL_INDEX = 3;
    public static final int RIGHTL_INDEX = 4;

    public static final double RED = 0.28;
    public static final double ORANGE = 0.333;
    public static final double YELLOW = 0.388;
    public static final double GREEN = 0.5;
    public static final double BLUE = 0.611;
    public static final double PURPLE = 0.72;
    public static final double BLACK = 0.0;
    public static final double WHITE = 1.0;

    /* --------------------------- AUTO CONSTANTS --------------------------- */
    //ALL CONSTANTS ARE BASED ON BLUE SIDE, ALL HEADING CONSTANTS ARE IN DEGREES, ALL TIME IS IN MILLISECONDS
    //CLOSE
    public static double CSHOOT_SPOT_X = -36.0;
    public static double CSHOOT_SPOT_Y = -28.0;

    public static double CSHOOT_DEGREES = -90.0;

    public static double CPICKUP1_X = -16.0;
    public static double CPICKUP1_Y = -56.0;
    public static double CBLUEPICKUP1_Y = -56.0;
    public static double CREDPICKUP1_Y = -50.0;
    public static double CPICKUP1_DEGREES = 0.0;

    public static double CPICKUP2_X = 8.0;
    public static double CPICKUP2_Y = -52.0;
    public static double CBLUEPICKUP2_Y = -52.0;
    public static double CREDPICKUP2_Y = -38.0;
    public static double CPICKUP2_DEGREES = 0.0;

    public static double CEND_X = -52.0;
    public static double CEND_Y = -20.0;
    public static double CEND_DEGREES = -90.0;


    //FAR
    public static double FPICKUP1_X = 72 - Constants.BOT_WIDTH/2.0;
    public static double FPICKUP1_Y = -72.0 + Constants.BOT_WIDTH/2.0;
    public static double FPICKUP1_DEGREES = -90.0;

    public static double FSHOOT_SPOT_X = 72 - Constants.BOT_WIDTH/2.0;
    public static double FSHOOT_SPOT_Y = -Constants.BOT_WIDTH/2.0;
    public static double FSHOOT_DEGREES = -90.0;

    public static double FENDY = -48.0;
    public static double FENDX = 72.0 - Constants.BOT_WIDTH/2.0;
    public static double FEND_DEGREES = -90.0;


    public static long SHOOTER_TIMEOUT = 1500;





    /* ---------------------------- Physical Robot Information ---------------------------- */
    public static final String LL = "limelight";
    //Control Hub Ports
            //Motor Ports
    public static final String CM0 = "leftFront";
    public static final String CM1 = "leftBack";
    public static final String CM2 = "rightBack";
    public static final String CM3 = "rightFront";

        //Analog Inputs
    public static final String CAI0 = "";
    public static final String CAI1 = "";
    public static final String CAI2 = "axonIn";
    public static final String CAI3 = "";


    //Servo Ports
    public static final String CS0 = "hoodAim";
    public static final String CS1 = "kickerRotate";
    public static final String CS2 = "turretAim";
    public static final String CS3 = "kickerSpin";
    public static final String CS4 = "sFeed";
    public static final String CS5 = "spindexer";

        //I2C Busses
    public static final String CBUS0 = "imu";
    public static final String CBUS1 = "pinpoint";
    public static final String CBUS2 = "color1Bay1";
    public static final String CBUS3 = "color2Bay1";

        // Digital Inputs
    public static final String CDI0 = "topBreak";
    public static final String CDI1 = "";
    public static final String CDI2 = "intakeBB";
    public static final String CDI3 = "";
    public static final String CDI4 = "";
    public static final String CDI5 = "";
    public static final String CDI6 = "";
    public static final String CDI7 = "";



    //Expansion Hub Ports
        //Motor Ports
    public static final String EM0 = "tFeed";
    public static final String EM1 = "intakeMotor";
    public static final String EM2 = "shooterRight";
    public static final String EM3 = "shooterLeft";

        //Analog Inputs
    public static final String EAI0 = "";
    public static final String EAI1 = "";
    public static final String EAI2 = "";
    public static final String EAI3 = "";


        //Servo Ports
    public static final String ES0 = ""; //old CR Servo transfer
    public static final String ES1 = "rightLight";
    public static final String ES2 = "leftLight";
    public static final String ES3 = "botLight";
    public static final String ES4 = "midLight";
    public static final String ES5 = "topLight";

        //I2C Busses
    public static final String EBUS0 = "color1Bay3";
    public static final String EBUS1 = "color2Bay3";
    public static final String EBUS2 = "color1Bay2";
    public static final String EBUS3 = "color2Bay2";

        // Digital Inputs
    public static final String EDI0 = "";
    public static final String EDI1 = "";
    public static final String EDI2 = "";
    public static final String EDI3 = "";
    public static final String EDI4 = "";
    public static final String EDI5 = "";
    public static final String EDI6 = "";
    public static final String EDI7 = "";


    /* ---------------- BOT DIMENSIONS ---------------- */
    public static final double BOT_WIDTH = 17.0;
    public static final double BOT_LENGTH = 17.75;
    /**How far forward the Limelight is in inches*/
    public static final double LL_FORWARD_OFFSET = 0.18611088 * METER_TO_INCH;
    /**How far up the Limelight is in inches*/
    public static final double LL_UP_OFFSET = 0.31369 * METER_TO_INCH;


    /* TODO LIST
      - Do what the driver says :)
        ~ stack lights
        ~ auto can probably be better :(
        ~ see if turret tracking can get any better
     */
}
