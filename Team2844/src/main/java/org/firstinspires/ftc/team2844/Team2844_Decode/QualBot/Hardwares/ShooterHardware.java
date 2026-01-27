package org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Hardwares;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class ShooterHardware {
    /*
     * Must Haves
     */
    private LinearOpMode opMode_;

    /*
     * Motors
     */
    public DcMotorEx shooterMotor;
    public DcMotor intakeMotor;

    /*
     * Servos
     */
    private Servo blockSer;
    private Servo hoodSer;
    //Constants
    private final double OUT_POS = 0.2;
    private final double IN_POS = 0.0;

    private boolean servoClosed;


    /*
     * Sensors
     */

    //BeamBreaks (BB)
    private DigitalChannel BB0;
    private DigitalChannel BB1;
    private DigitalChannel BB2;
    private DigitalChannel BB3;
    private DigitalChannel gobuildaBB;
    private DigitalChannel gobuildaBB1;

    /*
     * Shooter Conversions
     */
    public final double ENCODER_TICS = 28;
    public double velThresh = 10;
    public final double VEL_BOTTOM_THRESH = 1.5;

    public double shooterVel = 30;
    public double hoodAim = 0.0;

    public PIDFCoefficients shooterCoefficients;
    //OG values P:10 I:3 D:0 F:0
    PIDFCoefficients newShooterCoefficients;

    public ShooterHardware(LinearOpMode opMode) {
        /*
         * must haves
         */
        opMode_ = opMode;

        /*
         * Hardware maps and config for motors
         */
        shooterMotor = opMode_.hardwareMap.get(DcMotorEx.class, "shooter");
        intakeMotor = opMode_.hardwareMap.get(DcMotor.class, "intake");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /*
         * Hardware maps and config for Servos
         */
        blockSer = opMode_.hardwareMap.get(Servo.class, "blockSer");
        closeServo();

        hoodSer = opMode_.hardwareMap.get(Servo.class, "hoodSer");





        /*
         * hardware maps and config for sensors
         */
        BB0 = opMode_.hardwareMap.get(DigitalChannel.class, "BB0");
        BB1 = opMode_.hardwareMap.get(DigitalChannel.class, "BB1");
        BB2 = opMode_.hardwareMap.get(DigitalChannel.class, "BB2");
        BB3 = opMode_.hardwareMap.get(DigitalChannel.class, "BB3");
        gobuildaBB = opMode_.hardwareMap.get(DigitalChannel.class, "gobuildaBB");
        gobuildaBB1 = opMode_.hardwareMap.get(DigitalChannel.class, "gobuildaBB1");

        shooterCoefficients = shooterMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        newShooterCoefficients = new PIDFCoefficients(3.6, 3.2, 2.8, 3.7);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newShooterCoefficients);

    }

    public ShooterHardware(LinearOpMode opMode, boolean auto){
        new ShooterHardware(opMode);
        velThresh = 1;
    }

    /*
     * Motor methods
     */

    /**
     * The Intake method turns on the intake motor which takes in balls and if the servo is open, will feed to the shooter.
     * @param power Takes a double from -1.0 to 1.0, numbers over 1 will be set to 1, numbers under -1 will be set to -1
     */
    public void intake(double power){
        double temp = power;
        if(Math.abs(temp) > 1){
            temp/=Math.abs(temp);
        }
        intakeMotor.setPower(temp);
        //shooterMotor.setPower(-0.15); //JAE add
    }

    /**
     * This method sets the power of the shooter motor and nothing else
     * @param power Takes a double from -1.0 to 1.0, numbers over 1 will be set to 1, numbers under -1 will be set to -1
     */
    public void setShootPower(double power){
        double temp = power;
        if(Math.abs(temp) > 1){
            temp/=Math.abs(temp);
        }
        shooterMotor.setPower(temp);
    }

    /**
     * This will open the servo that holds the balls from the shooter, and runs the intake so the balls are feed into the shooter
     */

    public void feed(){
        intake(1);
        openServo();
    }

    public void extake(double power){
        intake(-power);
        shooterMotor.setPower(-power); //JAE add
        openServo();
    }

    public void stopFeed(){
        intake(0.0);
        //shooterMotor.setPower(0); //jae
        closeServo();
    }

    /**
     * Sets shooter and intake to 0 and closes the servo to hold balls from shooting
     */
    public void stopShooter(){
        shooterMotor.setPower(0);
        stopFeed();
    }

    public double getShootVelocity(){
        return (shooterMotor.getVelocity()/ENCODER_TICS);
    }

    public void setShootVelocity(double velocity){
        shooterMotor.setVelocity(velocity*ENCODER_TICS);
    }

    public double getShootSpeed(double distance) {
        if (distance != -999) {
            shooterVel = ((0.1445058824*distance) + 25.99294118);
            //shooterVel = ((0.1824324324*distance) + 21.88378378);
            return shooterVel;
        } else {
            return 30;
        }
    }

    public double getShootPowerLINREG(double vel){
        return ((0.012034*vel) - 0.00615);
    }

    public double lastKnownSpeed(){
        return  shooterVel;
    }

    public double getHoodAim(double distance) {
        if (distance != -999) {
            hoodAim = ((.0038895*distance) + -0.111817);
            return hoodAim;
        } else {
            return 0.0;
        }
    }

    public double lastKnownAim(){
        return hoodAim;
    }

    public boolean withinVel(double targVel){
        return ((targVel) < getShootVelocity()) && (getShootVelocity() < (targVel + velThresh));
    }

    public boolean belowVel(double targVel){
        return (getShootVelocity() < (targVel - VEL_BOTTOM_THRESH));
    }

    /*
     * Servo methods
     */
    public void openServo(){
        blockSer.setPosition(IN_POS);
        servoClosed = false;
    }


    public void closeServo(){
        blockSer.setPosition(OUT_POS);
        servoClosed = true;
    }

    public void testServo(double pos){
        blockSer.setPosition(pos);
    }

    public boolean servoClosed(){return servoClosed;}

    public void aimHood(double pos){hoodSer.setPosition(pos);}

    public boolean oneBall(){
        return ((!BB0.getState() || !BB1.getState()) && gobuildaBB1.getState()) || (!BB2.getState() || !BB3.getState()) || gobuildaBB.getState();
    }

    public boolean twoBall(){
        boolean spot1 = ((!BB0.getState() || !BB1.getState()) && gobuildaBB1.getState());
        boolean spot2 = (!BB2.getState() || !BB3.getState());
        boolean spot3 =  gobuildaBB.getState();
        return  (spot1 && spot2) || (spot1 && spot3) || (spot2 && spot3);
    }

    public boolean threeBall(){
        return ((!BB0.getState() || !BB1.getState()) && gobuildaBB1.getState()) && (!BB2.getState() || !BB3.getState()) && gobuildaBB.getState();
    }
}