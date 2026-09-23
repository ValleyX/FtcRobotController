package org.firstinspires.ftc.team12841.teleop;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team12841.Drivers.ClimberDriver;
import org.firstinspires.ftc.team12841.Drivers.DroneLauncher;
import org.firstinspires.ftc.team12841.Drivers.IntakeDriver;
import org.firstinspires.ftc.team12841.Drivers.LiftDrive;
import org.firstinspires.ftc.team12841.Drivers.RobotHardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Timer;
import java.util.concurrent.TimeUnit;


//@Disabled
@TeleOp(name = "FullTeleOp")
public class FullTeleOp extends LinearOpMode {

    RobotHardware robot;
    LiftDrive liftDrive;
    IntakeDriver intakeDriver;
    ClimberDriver climberDriver;

    DroneLauncher droneLauncher;

    ElapsedTime teleopTimer;


    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = 2;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;
    boolean isBucketClosed = true;// Used to hold the data for a detected AprilTag


   public int waittime = 150;

    boolean targetFound     = false;    // Set to true when an AprilTag target is detected
    double  drive           = 0;        // Desired forward power/speed (-1 to +1)
    double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
    double  turn            = 0;        // Desired turning power/speed (-1 to +1)

    boolean babyMode = false;

    boolean bLiftInMotion = false;


//    //set lights



    @Override
    public void runOpMode() throws InterruptedException {


        // Initialize the Apriltag Detection process
       // initAprilTag();
        robot = new RobotHardware(this,true);
        intakeDriver = new IntakeDriver(robot);
        liftDrive = new LiftDrive(robot);
        climberDriver = new ClimberDriver(robot);
        droneLauncher = new DroneLauncher(robot);
        teleopTimer = new ElapsedTime();
        //robot.winkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
//
//        robot.blinkinLedDriver.setPattern(robot.redPattern);
//        robot.winkinLedDriver.setPattern(robot.redPattern);

        waitForStart();
        teleopTimer.reset();

       // if (isStopRequested()) return;

        //
        while (opModeIsActive()) {
/*
            //if bumper is pressed, then go to the april tag
            if (gamepad1.left_bumper) {

                aprilTagDrive();

            } else {
*/
                fieldCentricControl();
                if (babyMode)
                {
                    robot.winkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                } else  {
                    robot.winkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE);
                }
            if(teleopTimer.seconds() > 90 && teleopTimer.seconds() < 105){
                    //robot.winkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                if (babyMode)
                {
                    robot.winkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                } else  {
                    robot.winkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE);
                }

                }
                if(teleopTimer.seconds() > 105 && teleopTimer.seconds() < 115){
                    //robot.winkinLedDriver.setPattern(robot.endgamePattern);
                    robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
                    if (babyMode)
                    {
                        robot.winkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                    } else  {
                        robot.winkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE);
                    }

                }
                if(teleopTimer.seconds() > 115 && teleopTimer.seconds() < 120){
                //robot.winkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
                    robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
                    if (babyMode)
                    {
                        robot.winkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                    } else  {
                        robot.winkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE);
                    }

                 }
                if(teleopTimer.seconds() > 120){
                   // robot.winkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
                    robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
                    if (babyMode)
                    {
                        robot.winkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                    } else  {
                        robot.winkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE);
                    }

                }

 //           }

        }
    }

    //make the function for field centric driving
    public void fieldCentricControl () {


        //robot.leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //robot.leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);


        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x /* 1.1*/; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;


        if (gamepad1.x) {
            babyMode = !babyMode;
            sleep(200);
        }

        // Read inverse IMU heading, as the IMU heading is CW positive

        //Gets the bot heading by use Geting angles from imu and geting the yaw in degress from that
        //double botHeading = /*-Math.toRadians(robot.getNavXHeading());*/-robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double botHeading = -Math.toRadians(robot.imu.getAngularOrientation().firstAngle - robot.saveGryoAngle);


        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        if (babyMode) {
            frontLeftPower *= 0.25;
            backLeftPower *= 0.25;
            frontRightPower *= 0.25;
            backRightPower *= 0.25;

        }

        //if robot found tag, then skip this and go to tag
        robot.leftFrontDrive.setPower(frontLeftPower);
        robot.leftBackDrive.setPower(backLeftPower);
        robot.rightFrontDrive.setPower(frontRightPower);
        robot.rightBackDrive.setPower(backRightPower);


        telemetry.addData("rotX", rotX);
        telemetry.addData("rotY", rotY);
        telemetry.addData("front left power", frontLeftPower);
        telemetry.addData("front right power", frontRightPower);
        telemetry.addData("back left power", backLeftPower);
        telemetry.addData("back right power", backRightPower);
        telemetry.addData("denominator", denominator);
        telemetry.addData("baby", babyMode);


        //----------------------------------------------------------------------------------------------
        //AUXILIARY CONTROLS


        telemetry.addData("waittime", waittime);


        //Left Trigger = intake, Right Trigger = outtake
        if (gamepad1.left_trigger >= 0.1) {
            intakeDriver.intakeOn(true, 1);
        } else if (gamepad1.right_trigger >= 0.1) {
            intakeDriver.intakeOn(true, -1);
        } else {
            intakeDriver.intakeOn(false, 0);
        }


        //Drone Launcher Release Code
        if (gamepad2.x == true) {
            droneLauncher.launch(true);
        }



        telemetry.addData("right sticky", gamepad2.right_stick_y);
        telemetry.addData("Left Motor Pos", robot.liftMotorLeft.getCurrentPosition());
        telemetry.addData("Right Motor Pos", robot.liftMotorRight.getCurrentPosition());
        telemetry.addData("imu heading", Math.toDegrees(botHeading));
        robot.OpMode_.telemetry.addData("climber pos", robot.climbMotor.getCurrentPosition());


        //reset heading
        if (gamepad1.y) {
           robot.resetNavXHeading();

            sleep(200);
        }




        //open bucket if held
        if (gamepad2.right_trigger > .5) {
            if (isBucketClosed) {
                if (robot.liftMotorRight.getCurrentPosition() > 1500){
                    robot.bucketServo.setPosition(robot.BUCKET_OPEN+.02);
                } else {
                    robot.bucketServo.setPosition(robot.BUCKET_MIDDLE);
                    sleep(50);
                    robot.bucketServo.setPosition(robot.BUCKET_OPEN);
                    sleep(100);
                    isBucketClosed = false;
                }
            }
        } else {
            robot.bucketServo.setPosition(robot.BUCKET_CLOSED);
            isBucketClosed = true;
        }


        //manuel climber
        /*if (gamepad2.dpad_up){
            climberDriver.climberUp(1);
        } else if (gamepad2.dpad_down){
            climberDriver.climberDown(-1);
        } else {
            climberDriver.climberDown(0);
        }*/


        //quick release climber
        if (gamepad2.dpad_left) {
            climberDriver.releaseClimber();
        } else if (gamepad2.dpad_down) {
            climberDriver.climberDown(-1);
        } else if (gamepad2.dpad_up) {
            climberDriver.climberUp(1);
        } else {
            climberDriver.climberDown(0);
        }




/* current
        if (gamepad2.a) {// first tape mark liftheight
            liftDrive.liftToEncoderCount(657,.5,10,1000,true);
        }

        if (gamepad2.b) {// second tape mark lift height
            liftDrive.liftToEncoderCount(1150,.5,10,1000,true);
        }
*/

        //reset lift
        if ((gamepad2.right_bumper) && (!bLiftInMotion)) {
            liftDrive.liftToEncoderCount(0,1,50,1000,false);
            bLiftInMotion = true;
        }
        //level 1
        if ((gamepad2.a) && (!bLiftInMotion)) {// first tape mark liftheight
            liftDrive.liftToEncoderCount(657,1,10,1000,false);
            bLiftInMotion = true;
        }
        //level 2
        if ((gamepad2.b) && (!bLiftInMotion))  {// second tape mark lift height
            liftDrive.liftToEncoderCount(1150,1,10,1000,false);
            bLiftInMotion = true;
        }
        //level 3
        if ((gamepad2.y) && (!bLiftInMotion))  {// second tape mark lift height
            liftDrive.liftToEncoderCount(1800,1,10,1000,false);
            bLiftInMotion = true;
        }

        if(Math.abs(gamepad2.right_stick_y) > .1 && !bLiftInMotion)
        {

            liftDrive.moveLiftNoBounds(-gamepad2.right_stick_y);
        }else if (!bLiftInMotion){
            liftDrive.moveLift(0);
        }

        if (gamepad2.left_bumper){
            robot.liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.liftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if (bLiftInMotion)
        {
            bLiftInMotion = !liftDrive.isFinished();
        }
        telemetry.addData("Lift In Motion = ", bLiftInMotion);


//657 lift low
// 1150 lift middle


        telemetry.update();

    } //field centric drive thingy end bracket


    //APRIL TAG STUFF
//_________________________________________________________________________________________________________________-
    public void aprilTagDrive () {

        //reverses motors so that the FIRST provided code works correctly
        robot.rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        //sets initial status for variables
        targetFound = false;
        desiredTag  = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) &&
                    ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID))  ){
                targetFound = true;
                desiredTag = detection;
                break;  // don't look any further.
            } else {
                telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
            }
        }

        // Tell the driver what we see, and what to do.
        if (targetFound) {
            telemetry.addData(">","HOLD Left-Bumper to Drive to Target\n");
            telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
        } else {
            telemetry.addData(">","Drive using joysticks to find valid target\n");
        }

        // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
        if (gamepad1.left_bumper && targetFound) {

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double  rangeError      = (desiredTag.ftcPose.range - robot.DESIRED_DISTANCE);
            double  headingError    = desiredTag.ftcPose.bearing;
            double  yawError        = desiredTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive  = Range.clip(rangeError * robot.SPEED_GAIN, -robot.MAX_AUTO_SPEED, robot.MAX_AUTO_SPEED);
            turn   = Range.clip(headingError * robot.TURN_GAIN, -robot.MAX_AUTO_TURN, robot.MAX_AUTO_TURN) ;
            strafe = Range.clip(-yawError * robot.STRAFE_GAIN, -robot.MAX_AUTO_STRAFE, robot.MAX_AUTO_STRAFE);

            telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        } else {

            // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
            drive  = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
            strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
            turn   = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
            telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        }
        telemetry.update();

        // Apply desired axes motions to the drivetrain.
        moveRobot(drive, strafe, turn);
        sleep(10);

    }





    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        //prevents motor overwork
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        robot.leftFrontDrive.setPower(leftFrontPower);
        robot.rightFrontDrive.setPower(rightFrontPower);
        robot.leftBackDrive.setPower(leftBackPower);
        robot.rightBackDrive.setPower(rightBackPower);
    }

    /**
     * Initialize the AprilTag processor. *black magic*
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcamcolor"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }


} //TeleOp end bracket
