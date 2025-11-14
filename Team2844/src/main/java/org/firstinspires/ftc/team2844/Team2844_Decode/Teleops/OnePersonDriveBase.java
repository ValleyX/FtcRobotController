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

    //shoot test stuff
    boolean dPadUp = false;
    boolean dPadDown = false;
    double shooterSpeed = 0.5;
    double shooterVelocity = 30;
    double time = 0;
    double spinupTime = 0;
    boolean firstTime = true;

    //shooter
    boolean stopAutoShoot = true;
    boolean stopManShoot = true;

    //Intake
    boolean intaking = false;

    //hood shooter
    double hoodPos = 0.0;
    boolean dPadRight = false;
    boolean dPadLeft = false;



    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware = new RobotHardware(this);
        shooterHardware = new ShooterHardware(this);
        //limelightHardware = new LimelightHardware(this);
        OpMode_ = this;

        //If we end auto facing the back of the field (away from audience) then comment out this line
        robotHardware.resetImu();

        if(limelightHardware.getPipeline() == 1){
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

            //run intake if left trigger is pulled\
            if(gamepad1.left_trigger > 0.1) {
                shooterHardware.intake(gamepad1.left_trigger);
                intaking = true;
            } else if(gamepad1.right_trigger > 0.1) {
                shooterHardware.intake(-gamepad1.right_trigger);
                intaking = true;
            } else {
                intaking = false;
            }

            //shooter
            if(gamepad1.right_bumper){
                if(!aligned) {
//                    if(limelightHardware.getTx() == -999) {
//                        robotHardware.turnToEstimate(red);
//                    }
                    time = getRuntime();
                    aligned = true;
                }
                double limelightResult = limelightHardware.getTx();
                if(limelightResult != -999) {
                    robotHardware.alignFree(limelightResult);
                }
                stopAutoShoot = false;
                shooterHardware.setShootVelocity(shooterVelocity);
                if(shooterHardware.withinVel(shooterVelocity)) {
                    if(firstTime){
                        spinupTime = getRuntime() - time;
                        firstTime = false;
                    }
                    shooterHardware.feed();
                } else if(shooterHardware.belowVel(shooterVelocity)) {
                    shooterHardware.stopFeed();
                }
            } else {
                aligned = false;
                stopAutoShoot = true;
                firstTime = true;
            }

            if(gamepad1.left_bumper){
                stopManShoot = false;
                shooterHardware.shoot(shooterSpeed);
                shooterHardware.feed();
            } else {
                stopManShoot = true;
            }

            if(stopAutoShoot && stopManShoot && !intaking){
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
                    if(limelightHardware.getTx() == -999) {
                        robotHardware.turnToEstimate(red);
                    }
                    alignPress = true;
                }
            } else {
                 alignPress = false;
            }

            if(gamepad1.dpad_up){
                if(!dPadUp){
                    shooterSpeed += 0.01;
                    shooterVelocity += 1;
                    dPadUp = true;
                }
            } else {
                dPadUp = false;
            }

            if(gamepad1.dpad_down){
                if(!dPadDown){
                    shooterSpeed -= 0.01;
                    shooterVelocity -= 1;
                    dPadDown = true;
                }
            } else {
                dPadDown = false;
            }

            if(gamepad1.dpad_left){
                if(!dPadLeft){
                    hoodPos -= 0.01;
                    dPadLeft = true;
                }
            } else {
                dPadLeft = false;
            }

            if(gamepad1.dpad_right){
                if(!dPadRight){
                    hoodPos += 0.01;
                    dPadRight = true;
                }
            } else {
                dPadRight = false;
            }

            shooterHardware.aimHood(hoodPos);

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


            telemetry.addData("hood pos:", hoodPos);
            telemetry.addData("Shooter Speed", shooterSpeed);
            telemetry.addData("Velocity: ", shooterHardware.getShootVelocity());
            telemetry.addData("Spinup Time: ", spinupTime);
            telemetry.addData("Target Velocity: ", shooterVelocity);
            telemetry.addData("IMU (Degrees)", robotHardware.robotHeadingAngles());
            telemetry.addData("Limelight Tx", limelightHardware.getTx());
            telemetry.addData("LImelight TArea", limelightHardware.getTarea());
            telemetry.addData("LImelight Cam Z", limelightHardware.getBotCamZ());
            telemetry.addData("Servo Closed", shooterHardware.servoClosed());
            telemetry.addData("Babymode", babymode);
            telemetry.update();
        }
    }
}
