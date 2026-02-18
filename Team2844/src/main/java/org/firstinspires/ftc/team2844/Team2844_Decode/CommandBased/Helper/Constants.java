package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper;

public class Constants {

    //turret
    //24 tooth servo head to 130 turret teeth
    private static final double SERVO_TEETH = 130.0;
    private static final double TURRET_TEETH = 130.0; //r u sure its 130?
    private static final double TURRET_GEAR_RATIO = TURRET_TEETH/SERVO_TEETH;
    private static final double DEGREES_IN_FULL_SERVO_TURN = (360.0/TURRET_GEAR_RATIO);
    public static final double SERVO_DEGREE_TO_TURRET_DEGREE = TURRET_GEAR_RATIO;
//    public static final double DEGREES = 1/((5.0*24.0*360.0)/130.0);

    public static final double TURRET_THRESHHOLD = 1.0;

    public static final double MAX_DEGREE = 332.0;
    public static final double MIN_DEGREE = 10.0;

    public static final double MAX_TURN = MAX_DEGREE * SERVO_DEGREE_TO_TURRET_DEGREE;
    public static final double MIN_TURN = MIN_DEGREE * SERVO_DEGREE_TO_TURRET_DEGREE;
    public static final double TURN_TICK = 1;

    public static final double TURRET_OFFSET = 90.0;
    public static final double TURRET_GAIN = 0.009;
    public static final double GAIN_THRESH = 20.0;

    public static final double MIN_VOLTAGE = 0.22;
    public static final double MAX_VOLTAGE = 3.06;


    public static final double VELOCITY_THRESHHOLD = 250;
    public static final double MIN_VELOCITY = 1000;




    //Uptake
    public static final double MAX_KICKDOWN = 0.3;


    //spindexer
    public static final double SLOT_ZERO = 0.0;
    public static final double SLOT_ONE = 0.2/3;
    public static final double SLOT_TWO = 2*(0.2/3);

    public static final double SLOT_ZERO_LOOPED_ONE = 0.2;
    public static final double SLOT_ONE_LOOPED_ONE = 4*(0.2/3);
    public static final double SLOT_TWO_LOOPED_ONE = 5*(0.2/3);

    public static final double[] SLOT_ARRAY = {SLOT_ZERO, SLOT_ONE, SLOT_TWO, SLOT_ZERO_LOOPED_ONE, SLOT_ONE_LOOPED_ONE, SLOT_TWO_LOOPED_ONE};

    public static final int MIN_COLOR_SUM = 900;
    public static final int MIN_ALPHA = 300;

    public static final int PURPLE = 1;
    public static final int GREEN = 0;

    /** If all colors added are equal to this number, they are sortable with one green and two purple*/
    public static final int SORTABLE = PURPLE + PURPLE;
    public static final int UNKNOWN_COLOR = 999;

    public static final int PATTERN_PPG = 2110;
    public static final int PATTERN_PGP = 2101;
    public static final int PATTERN_GPP = 2011;

    //Limelight
    public static final int NO_LL = -999;
    public static final String LL = "limelight";

    public static final int BLUE_PIPELINE = 0;
    public static final int RED_PIPELINE = 1;

    public static final int BLUE_PIPELINE_MOTIF = 2;
    public static final int RED_PIPELINE_MOTIF = 3;

    public static final double BLUE_APRILTAG_X = 1.482 * 39.3701;
    public static final double BLUE_APRILTAG_Y = 1.413 * 39.3701;

    public static final double RED_APRILTAG_X = 1.482 * 39.3701;
    public static final double RED_APRILTAG_Y = -1.413 * 39.3701;

    //Pinpoint
    public static final double NO_PP = -999.0;

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
    public static final String ES0 = "";
    public static final String ES1 = "";
    public static final String ES2 = "";
    public static final String ES3 = "";
    public static final String ES4 = "";
    public static final String ES5 = "";

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

    public static final double BOT_WIDTH = 17.0;
    public static final double BOT_LENGTH = 17.75;
    public static final double STRAFE_CORRECTION = 1.35;



    //Intake
    public static final double INTAKE_SPEED = 1.0;

    public static final double SLOW_TFEED = 0.1;

    /* TODO LIST
      - Get Intake Spindexer working with new beambreak (stop intaking when full)
      - get the spindexer to spin the feeds while spinning

      - Get Shooter Velocity Working
         * get the linreg for shooter
         * get hood linreg working

      - get pose tracking with the odometry pods
        * get distance from limelight
      - get Metatags with the limelight
        * update pinpoint pose with metatag pos

      After rebuild of turret
      - get axon to track degrees of turret
      - change auto-alin to continuous

      After Mecanum
      - Change drive cmd
      - get roadrunner working

      - Auto!!!

     */
}
