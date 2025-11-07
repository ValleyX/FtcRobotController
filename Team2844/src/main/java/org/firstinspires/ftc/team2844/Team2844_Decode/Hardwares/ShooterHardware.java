package org.firstinspires.ftc.team2844.Team2844_Decode.Hardwares;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    private final double CLOSE_POS = 0.1;
    private final double OPEN_POS = 0.0;


    /*
     * Sensors
     */

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
        /*
         * Hardware maps and config for Servos
         */
        blockSer = opMode_.hardwareMap.get(Servo.class, "blockSer");
        closeServo();

        hoodSer = opMode_.hardwareMap.get(Servo.class, "hoodSer");



        /*
         * hardware maps and config for sensors
         */


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
    }

    /**
     * This method sets the power of the shooter motor
     * @param power Takes a double from -1.0 to 1.0, numbers over 1 will be set to 1, numbers under -1 will be set to -1
     */
    public void shoot(double power){
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
        openServo();
        intake(0.7);
    }

    /**
     * Sets shooter and intake to 0 and closes the servo to hold balls from shooting
     */
    public void stopShooter(){
        shooterMotor.setPower(0);
        intake(0.0);
        closeServo();
    }

    /*
     * Servo methods
     */
    public void openServo(){blockSer.setPosition(OPEN_POS);}
    public void closeServo(){blockSer.setPosition(CLOSE_POS);}

    public void aimHood(double pos){hoodSer.setPosition(pos);}


}