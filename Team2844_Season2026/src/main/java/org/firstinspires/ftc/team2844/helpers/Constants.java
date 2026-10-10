package org.firstinspires.ftc.team2844.helpers;

public class Constants {

    /* ---------- VELOCITY PIDS ---------- */
    public static final double NP = 1.0;
    public static final double NI = 1.0;
    public static final double ND = 1.0;
    public static final double NS = 1.0;
    public static final double NV = 1.0;


    public static final double PP = 1.0;
    public static final double PI = 1.0;
    public static final double PD = 1.0;
    public static final double PS = 1.0;
    public static final double PV = 1.0;


    /* ---------- MISSING INFO ---------- */
    public static final double NO_HEADING = -999;
    public static final double NO_POS = -999;
    public static final double NO_LL = -999;


    /* ---------- COORDINATE DATA ---------- */
    public static double targetX = Constants.NO_POS;
    public static double targetY = Constants.NO_POS;

    public static final double BLUE_AUDIENCE_X = 89.0;
    public static final double BLUE_AUDIENCE_Y = 53.0;
    public static final double BLUE_BACK_X = 98.0;
    public static final double BLUE_BACK_Y = 91.0;
    public static final double RED_AUDIENCE_X = 55.0;
    public static final double RED_AUDIENCE_Y = 53.0;
    public static final double RED_BACK_X = 55.0;
    public static final double RED_BACK_Y = 91.0;

    /* ---------- ID CONSTANTS ---------- */
    public static final int BLUE_AUDIENCE = 0;
    public static final int BLUE_BACK = 1;
    public static final int RED_AUDIENCE = 2;
    public static final int RED_BACK = 3;



    /* ---------- Set Speeds and Deadbands ---------- */

    public static double intakeSpeed = 1.0;

    public static double STICK_DEADBAND = 0.05;


    /**
     * This method changes the values for the goal's field coordinates depending on case given
     * The field coordinates are rough estimates of where the cell is above when tipped up.
     * @param target 0- Blue Audience 1- Blue Back 2- Red Audience 3- Red Back
     *
     */
    public static void changeTarget(int target) {
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
    public static final double XPOD_OFFSET = 0.0;
    public static final double YPOD_OFFSET = 0.0;


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


    public static final String CHS0 = "";
    public static final String CHS1 = "";
    public static final String CHS2 = "";
    public static final String CHS3 = "";
    public static final String CHS4 = "";
    public static final String CHS5 = "";

    public static final String CHI2C0 = "";
    public static final String CHI2C1 = "pinpoint";
    public static final String CHI2C2 = "";
    public static final String CHI2C3 = "";

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
    public static final String EHM1 = "";
    public static final String EHM2 = "nectarShooter";
    public static final String EHM3 = "pollenShooter";


    public static final String EHS0 = "";
    public static final String EHS1 = "";
    public static final String EHS2 = "";
    public static final String EHS3 = "";
    public static final String EHS4 = "";
    public static final String EHS5 = "";

    public static final String EHI2C0 = "";
    public static final String EHI2C1 = "";
    public static final String EHI2C2 = "";
    public static final String EHI2C3 = "";

    public static final String EHDD0 = "";
    public static final String EHDD1 = "";
    public static final String EHDD2 = "";
    public static final String EHDD3 = "";
    public static final String EHDD4 = "";
    public static final String EHDD5 = "";
    public static final String EHDD6 = "";
    public static final String EHDD7 = "";

}
