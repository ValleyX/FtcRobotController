package org.firstinspires.ftc.team2844.Team2844_Decode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team2844.Team2844_Decode.Hardwares.LimelightHardware;
import org.firstinspires.ftc.team2844.Team2844_Decode.Hardwares.RobotHardware;
import org.firstinspires.ftc.team2844.Team2844_Decode.Hardwares.ShooterHardware;

@Disabled
public class OnePersonDriveBase extends LinearOpMode {
    RobotHardware robotHardware;
    ShooterHardware shooterHardware;
    LimelightHardware limelightHardware;
    LinearOpMode OpMode_;


    //feild centric vars
    double x;
    double y;
    double rx;
    double botHeading;

    //babymode stuff
    boolean pressingStick = false;
    boolean babymode = false;
    private final double BABY_MULT = 0.25;

    //What am I?
    boolean red = false;

    //make sure it only aligns once per trigger pull
    boolean aligned = false;
    //align toggle stuff
    boolean alignPress = false;
    boolean autoAlign = false;



    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware = new RobotHardware(this);
        shooterHardware = new ShooterHardware(this);
        limelightHardware = new LimelightHardware(this);
        OpMode_ = this;

        //If we end auto facing the back of the field (away from audience) then comment out this line
        robotHardware.resetImu();

        if(limelightHardware.getPipeline() == 0){
            red = true;
        }

        waitForStart();


        while (opModeIsActive()) {

            x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;
            y = -gamepad1.left_stick_y;

            //botHeading = robotHardware.robotHeadingRadians();
            botHeading = 0;

            //code for field centric (Idk how it works, pretty sure it's magic or makes triangles or something)
            //REMEMBER IT USES RADIANS
            telemetry.addData("botHeading", botHeading);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            if(babymode){
                frontLeftPower = frontLeftPower*BABY_MULT;
                backLeftPower = backLeftPower*BABY_MULT;
                backRightPower = backRightPower*BABY_MULT;
                frontRightPower = frontRightPower*BABY_MULT;
            }

            robotHardware.powerMotors(frontLeftPower, backLeftPower, backRightPower, frontRightPower);

            //run intake if left trigger is pulled
            shooterHardware.intake(gamepad1.left_trigger);
            //run extake if right trigger is pulled
            shooterHardware.intake(-gamepad1.right_trigger);

            //shooter
            if(gamepad1.left_trigger>0.1){
                if(!aligned) {
                    robotHardware.turnToEstimate(red);
                    aligned = true;
                }
                double limelightResult = limelightHardware.getTx();
                if(limelightResult != -999) {
                    robotHardware.alignFree(limelightResult);
                }
                shooterHardware.shoot(0.6);
                shooterHardware.feed();
            } else {
                aligned = false;
                shooterHardware.stopShooter();
            }

            //buttons
            if(gamepad1.right_stick_button){
                if(!pressingStick){
                    babymode = !babymode;
                    pressingStick = true;
                }
            } else {
                pressingStick = false;
            }

            if(gamepad1.a){
                if(!alignPress){
                    autoAlign = !autoAlign;
                    robotHardware.turnToEstimate(red);
                    alignPress = true;
                }
            } else {
                 alignPress = false;
            }

            if(autoAlign){
                double limelightResult = limelightHardware.getTx();
                if(limelightResult != -999) {
                    robotHardware.alignFree(limelightResult);
                }
            }

//            if(gamepad1.a){
//                robotHardware.turnToEstimate(red);
//                aligned = true;
//                double limelightResult = limelightHardware.getTx();
//                if(limelightResult != -999) {
//                    robotHardware.alignFree(limelightResult);
//                }
//            }

            //supposed to be middle logitech button; resets the Imu
            if(gamepad1.guide){
                robotHardware.resetImu();
            }
        }
    }
}
