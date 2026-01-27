package org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Hardwares.LimelightHardware;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Hardwares.RobotHardware;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Hardwares.ShooterHardware;

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

    final double rotCorrection = 1.0;
    final double strafeCorrection = 1.45;

    //babymode stuff
    boolean pressingStick = false;
    boolean babymode = false;
    private final double BABY_MULT = 0.4;
    boolean dUp = false;
    boolean dDown = false;

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
    boolean timeBool = true;

    //shooter
    boolean stopAutoShoot = true;
    boolean stopManShoot = true;
    boolean shooterAlign = false;

    //Intake
    boolean intaking = false;
    boolean full = false;

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

        PIDFCoefficients shooterCoefficients = shooterHardware.shooterMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients newShooterCoefficients = new PIDFCoefficients(40, 0, 15, 25);
        shooterHardware.shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newShooterCoefficients);

        //If we end auto facing the back of the field (away from audience) then comment out this line
        robotHardware.resetImu();

        if(limelightHardware.getPipeline() == 1){
            red = true;
        }

        waitForStart();


        while (opModeIsActive()) {

            limelightHardware.updateIMU(robotHardware.robotHeadingAngles());

            x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x * rotCorrection;
            y = -gamepad1.left_stick_y;

            botHeading = robotHardware.robotHeadingRadians();
            //botHeading = 0;

            //code for field centric (Idk how it works, pretty sure it's magic or makes triangles or something)
            //REMEMBER IT USES RADIANS
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + (rotX * strafeCorrection) + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - (rotX * strafeCorrection) - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            if(babymode){
                frontLeftPower = frontLeftPower*BABY_MULT;
                backLeftPower = backLeftPower*BABY_MULT;
                backRightPower = backRightPower*BABY_MULT;
                frontRightPower = frontRightPower*BABY_MULT;
            }


            robotHardware.addDrivePower(frontLeftPower, backLeftPower, backRightPower, frontRightPower);

            full = shooterHardware.threeBall();

            //run intake if left trigger is pulled\
            if(gamepad1.left_trigger > 0.1 && !full) {
                shooterHardware.intake(gamepad1.left_trigger);
                intaking = true;
                shooterHardware.setShootPower(-0.15);//jae
            } else if(gamepad1.right_trigger > 0.1) {
                shooterHardware.intake(-gamepad1.right_trigger);
                intaking = true;
            //} else if(gamepad1.dpad_down){
                //if(!dPadDown){
                  //  intaking = true;
                    //shooterHardware.extake(1.0);

                    //dPadDown = true;
               // }
            } else {
                intaking = false;
                //dPadDown = false;
                //shooterHardware.shooterMotor.setPower(0);//jae
            }

            if(gamepad1.b){
                shooterVelocity = shooterHardware.lastKnownSpeed();
                hoodPos = shooterHardware.lastKnownAim();
            } else {
                shooterVelocity = shooterHardware.getShootSpeed(limelightHardware.getBotDis());
                hoodPos = shooterHardware.getHoodAim(limelightHardware.getBotDis());
            }

            //shooter
            if(gamepad1.right_bumper){
                shooterAlign = true;
                stopAutoShoot = false;
                shooterHardware.aimHood(hoodPos);
                shooterHardware.setShootVelocity(shooterVelocity);

                if(timeBool) {
                    time = getRuntime();
                    timeBool = false;
                }

                if(shooterHardware.withinVel(shooterVelocity)) {
                    if(firstTime){
                        spinupTime = getRuntime() - time;
                        firstTime = false;
                    }
                    shooterHardware.setShootPower(shooterHardware.getShootPowerLINREG(shooterVelocity));
                    shooterHardware.feed();
                } else if(shooterHardware.belowVel(shooterVelocity)) {
                    shooterHardware.stopFeed();
                }
            } else {
                stopAutoShoot = true;
                shooterAlign = false;
                firstTime = true;
                timeBool = true;
            }

//            if(gamepad1.left_bumper){
//                stopManShoot = false;
//                shooterHardware.shoot(shooterSpeed);
//                shooterHardware.feed();
//            } else {
//                stopManShoot = true;
//            }

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
                    if(limelightHardware.getTx() == -999){
                        //robotHardware.turnToEstimate(red);
                    } else {
                        robotHardware.addAlignPower(0,0,0,0);
                    }
                    alignPress = true;
                }
            } else {
                 alignPress = false;
            }


            if(gamepad1.dpad_up){
                if(!dUp) {
                    shooterVelocity += .5;
                    dUp = !dUp;
                }
            } else {
                dUp = false;
            }

            if(gamepad1.dpad_down){
                if(!dDown) {
                    shooterVelocity -= .5;
                    dDown = !dDown;
                }
            } else {
                dDown = false;
            }

//            if(gamepad1.dpad_up){
//                if(!dPadUp){
//                    //shooterVelocity += 1;
//                    dPadUp = true;
//                }
//            } else {
//                dPadUp = false;
//            }
//
//            if(gamepad1.dpad_down){
//                if(!dPadDown){
//                    //shooterVelocity -= 1;
//                    dPadDown = true;
//                }
//            } else {
//                dPadDown = false;
//            }
//
//            if(gamepad1.dpad_left){
//                if(!dPadLeft){
//
//                    dPadLeft = true;
//                }
//            } else {
//                dPadLeft = false;
//            }
//
//            if(gamepad1.dpad_right){
//                if(!dPadRight){
//                    dPadRight = true;
//                }
//            } else {
//                dPadRight = false;
//            }

            if(autoAlign || shooterAlign){
                double limelightResult = limelightHardware.getTx();
                if(limelightResult != -999) {
                    robotHardware.alignFree(limelightResult);
                }
            } else {
                robotHardware.addAlignPower(0,0,0,0);
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



            telemetry.addData("(at least) One Ball: ", shooterHardware.oneBall());
            telemetry.addData("(at least) Two Balls: ", shooterHardware.twoBall());
            telemetry.addData("all three balls", full);
            telemetry.addData("Velocity: ", shooterHardware.getShootVelocity());
            telemetry.addData("Shooter Power", shooterHardware.shooterMotor.getPower());

            telemetry.addData("Spinup Time: ", spinupTime);
            telemetry.addData("Target Velocity: ", shooterVelocity);
            telemetry.addData("IMU (Degrees)", robotHardware.robotHeadingAngles());
            telemetry.addData("Limelight Tx", limelightHardware.getTx());
            telemetry.addData("Distance from tag", limelightHardware.getBotDis());
            telemetry.addData("Servo Closed", shooterHardware.servoClosed());
            telemetry.addData("Babymode", babymode);
            telemetry.addData("Shooter P:", shooterHardware.shooterCoefficients.p);
            telemetry.addData("Shooter I:", shooterHardware.shooterCoefficients.i);
            telemetry.addData("Shooter D:", shooterHardware.shooterCoefficients.d);
            telemetry.addData("Shooter F:", shooterHardware.shooterCoefficients.f);

            telemetry.addData("Front Left", frontLeftPower);
            telemetry.addData("Back Left", backLeftPower);
            telemetry.addData("Front Right:", frontRightPower);
            telemetry.addData("Back Right", backRightPower);
            telemetry.update();
        }
    }
}
