package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased;

public class Constants {

    //This is me testing pushing code from my home computer

    //turret
    //24 tooth servo head to 130 turret teeth
    private static final double SERVO_TEETH = 52.0;
    private static final double TURRET_TEETH = 130.0; //r u sure its 130?
    private static final double TURRET_GEAR_RATIO = TURRET_TEETH/SERVO_TEETH;
    private static final double DEGREES_IN_FULL_SERVO_TURN = (360.0/TURRET_GEAR_RATIO);
    public static final double SERVO_DEGREE_TO_TURRET_DEGREE = TURRET_GEAR_RATIO;
//    public static final double DEGREES = 1/((5.0*24.0*360.0)/130.0);

    public static final double TURRET_THRESHHOLD = 1.0;

    public static final double MAX_TURN = 332* SERVO_DEGREE_TO_TURRET_DEGREE;
    public static final double MIN_TURN = 0.0;
    public static final double TURN_TICK = 1;

    public static final double TURRET_OFFSET = 90.0;
    public static final double TURRET_GAIN = 0.04;







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

    public static final int PURPLE = 1;
    public static final int GREEN = 0;
    public static final int UNKNOWN_COLOR = 999;

    //Limelight
    public static final int NO_LL = -999;
    public static final String LL = "limelight";

    //Pinpoint
    public static final double NO_PP = -999.0;
    public static final double X_OFFSET = 4.75;
    public static final double Y_OFFSET = 5.5;

    //Control Hub Ports
        //Motor Ports
    public static final String CM0 = "frontLeft";
    public static final String CM1 = "backLeft";
    public static final String CM2 = "backRight";
    public static final String CM3 = "frontRight";

        //Analog Inputs
    public static final String CAI0 = "";
    public static final String CAI1 = "";
    public static final String CAI2 = "";
    public static final String CAI3 = "";


    //Servo Ports
    public static final String CS0 = "hoodAim";
    public static final String CS1 = "kickerRotate";
    public static final String CS2 = "turretAim";
    public static final String CS3 = "kickerSpin";
    public static final String CS4 = "sFeed";
    public static final String CS5 = "spindexer";

        //I2C Busses
    public static final String CBUS0 = "pinpointIMU";
    public static final String CBUS1 = "turretIMU";
    public static final String CBUS2 = "color1Bay1";
    public static final String CBUS3 = "color2Bay1";

        // Digital Inputs
    public static final String CDI0 = "";
    public static final String CDI1 = "topBreak";
    public static final String CDI2 = "";
    public static final String CDI3 = "intakeBB";
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
    public static final String EAI0 = "axonIn";
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



    //Intake
    public static final double INTAKE_SPEED = 0.9;

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
