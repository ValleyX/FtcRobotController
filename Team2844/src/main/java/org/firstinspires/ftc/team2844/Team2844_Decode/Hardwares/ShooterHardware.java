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
    public void intake(double power){
        double temp = power;
        if(temp > 1){
            temp/=temp;
        }
        intakeMotor.setPower(temp);
    }

    public void shoot(double power){
        openServo();
        double temp = power;
        if(temp > 1){
            temp/=temp;
        }
        shooterMotor.setPower(temp);
    }

    /*
     * Servo methods
     */
    public void openServo(){blockSer.setPosition(OPEN_POS);}
    public void closeServo(){blockSer.setPosition(CLOSE_POS);}

    public void aimHood(double pos){hoodSer.setPosition(pos);}


}