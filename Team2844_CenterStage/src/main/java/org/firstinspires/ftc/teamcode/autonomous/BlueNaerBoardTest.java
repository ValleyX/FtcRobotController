package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Drivers.AprilTag;
import org.firstinspires.ftc.teamcode.Drivers.ClimberDriver;
import org.firstinspires.ftc.teamcode.Drivers.GyroDrive;
import org.firstinspires.ftc.teamcode.Drivers.LiftDrive;
import org.firstinspires.ftc.teamcode.Drivers.RobotHardware;

import org.firstinspires.ftc.teamcode.Drivers.OdometryDrive;
import org.firstinspires.ftc.teamcode.testcode.GyroDriveTest;
import org.firstinspires.ftc.teamcode.testcode.RobotHardwareTestVersion;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

//robot must pe put in the same place to relatively same position per game to be consistant
//Position distance from corner near board is about 48 inches
@Disabled
@Autonomous(name="AutoBlueNearBoardtest")
public class BlueNaerBoardTest extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {

        //Using test hardware for now because test hardware has properties needed for practice
        RobotHardwareTestVersion robot = new RobotHardwareTestVersion(this,true); // use true if it is blueside

        GyroDriveTest gyroDrive = new GyroDriveTest(robot);//sets up Drives
        //LiftDrive liftDrive = new LiftDrive(robot);


        RobotHardwareTestVersion.CenterStagePipeline.DetectionPosition position = RobotHardwareTestVersion.CenterStagePipeline.DetectionPosition.Left; // position robot detects
        AprilTagDetection desiredTag = null;
        AprilTag aprilTag;

        int aprilTagID = 2;
        double drive = 0;
        double turn = 0;
        double strafe = 0;
        double distanceToBoard = 0;
        double startingDistanceFromBoard = 49;

        while(opModeInInit()){
            //to tell user what values the camera sees
            telemetry.addData("r value", robot.pipeline.avgR); // Telemetry for the user to see the avg values of blue and red for the boxes in the camera
            telemetry.addData("b value", robot.pipeline.avgB);
            telemetry.addData("r2 value", robot.pipeline.avg2R);
            telemetry.addData("b2 value", robot.pipeline.avg2B);
            telemetry.addData("r3 value", robot.pipeline.avg3R);
            telemetry.addData("b3 value", robot.pipeline.avg3B);

            telemetry.addData("teamProp position", robot.pipeline.position); // Telling the user what box it thinks the custom made game piece

            telemetry.update();

            position = robot.pipeline.position; //updating position to what the robot detects


        }
        if(position == robot.pipeline.position.Left){
            //go grab pixel

            //Code for parking
            gyroDrive.driveStraight(0.5,30,0);
            gyroDrive.turnToHeading(0.5,-90);
            gyroDrive.driveStraight(0.5,6,-90); //To push item
            gyroDrive.driveStraight(0.5,-6,-90);
            gyroDrive.turnToHeading(0.5,0);
            gyroDrive.driveStraight(0.5,-12,0);

            gyroDrive.turnToHeading(0.5,-90); //Turning to board and driving to board
            gyroDrive.driveStraight(0.5,30,-90);
            gyroDrive.turnToHeading(0.4,-90);
            sleep(1000); //-------------- sleep to put lift pixel here
            //drive to parking
            gyroDrive.turnToHeading(0.5,0);
            gyroDrive.driveStraight(0.6,-15,0);
            gyroDrive.turnToHeading(0.4,-90);
            gyroDrive.driveStraight(0.5,8,-90);


        }

        //Objective of  this code is to GyroscopeDrive to drive up to the April tag and place pixel then to park, later want to use april tag
        if(position == robot.pipeline.position.Middle){

            //push pixel out of the way
            gyroDrive.driveStraight(0.5,33,0);
            gyroDrive.driveStraight(0.4,-7,0);

            //turning and moving toward board
            gyroDrive.turnToHeading(0.3,-90);
            gyroDrive.driveStraight(0.5,33.5,-90);
            gyroDrive.turnToHeading(0.3,-90); //making sure robot is in place
            sleep(1000); //waiting at board ---------- place lift code

            //drive to parking area
            gyroDrive.turnToHeading(0.4,0);
            gyroDrive.driveStraight(0.5,-20,0);
            gyroDrive.turnToHeading(0.4,-90);
            gyroDrive.driveStraight(0.5,10,-90);



        }
        if(position == robot.pipeline.position.Right){

            //go bump pixel
            gyroDrive.driveStraight(0.6,30,0);
            gyroDrive.turnToHeading(0.5,90);
            gyroDrive.driveStraight(0.5,5,90); //Pushing game piece out of the way.
            gyroDrive.driveStraight(0.5,-5,90);
            //move to board
            gyroDrive.turnToHeading(0.4,0);
            gyroDrive.driveStraight(0.5,4.5,0);
            gyroDrive.turnToHeading(0.4,-90);
            gyroDrive.driveStraight(0.6,34,-90);
            gyroDrive.turnToHeading(0.4,-90);
            sleep(2000); //waiting at board

            //move to parking place
            gyroDrive.turnToHeading(0.5,0);
            gyroDrive.driveStraight(0.5,-31,0);
            gyroDrive.turnToHeading(0.5,-90);
            gyroDrive.driveStraight(0.5,10,-90);



        }
//        ctrl unslash to do
//
//        robot.switchableWebcam.setPipeline(null); // Turning off camera
//        robot.switchableWebcam.closeCameraDevice();
//        sleep(2000); // play with tested value
//
//
//        aprilTag = new AprilTag(robot);
//        aprilTag.initAprilTag(robot);
//
//        aprilTag.setManualExposure(robot,6,350);
//
//        waitForStart();
//        robot.imu.resetYaw();
//
//
//       // desiredTag = aprilTag.aprilTagDetected(AprilTagID);
//
//
//     //   robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Making motors run off the encoders
//    //    robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//     //   robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//     //   robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//
//
//        gyroDrive.driveStraight(0.5,24,0); //robot driving up to board
//        sleep(100);
//        gyroDrive.turnToHeading(0.5,-90); //robot turning toward board
//        //sleep(1000);
//        //gyroDrive.holdHeading(0.5,-90,1000); //TODO hold heading needs to be debugged
//
//
//        double rangeError;
//        double headingError;
//        double yawError;
//        //desiredTag.ftcPose = new AprilTagPoseFtc(3,3);
//        Boolean found = false;
//        Boolean targetReached = false;
//
//        boolean notRun = true;
//
//         */
//        /*
//        while (opModeIsActive() && !targetReached){
//           // if(aprilTag.isAprilTagDetected(AprilTagID)){
//                if ((desiredTag = aprilTag.aprilTagDetected(robot,aprilTagID)) != null)
//                {
//                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
//                    rangeError = (desiredTag.ftcPose.range - robot.DESIRED_DISTANCE);
//                    headingError = desiredTag.ftcPose.bearing;
//                    yawError = desiredTag.ftcPose.yaw;
//
//
//                    drive = Range.clip(rangeError * robot.SPEED_GAIN, -robot.MAX_AUTO_SPEED, robot.MAX_AUTO_SPEED);
//                    turn = Range.clip(headingError * robot.TURN_GAIN, -robot.MAX_AUTO_TURN, robot.MAX_AUTO_TURN);
//                    strafe = Range.clip(-yawError * robot.STRAFE_GAIN, -robot.MAX_AUTO_STRAFE, robot.MAX_AUTO_STRAFE);
//
//                    telemetry.addData("AprilTagID: ","range %d",aprilTagID);
//                    telemetry.addData("Auto","range %5.2f, heading %5.2f, yaw %5.2f ", rangeError, headingError, yawError);
//                    telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f rangeError %5.2f", drive, strafe, turn, rangeError);
//                    System.out.printf("ValleyX: Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
//                     telemetry.update();
//
//                    if((rangeError < 0.3) && (turn > 0.5) && notRun){
//                        // gyroDrive.turnToHeading(0.5,-90);
//                        //  targetReached = true;
//
//
//                       // if (robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > 3) {
//                            gyroDrive.turnToHeading(0.5, -90);
//                            notRun = false;
//
//                     //       continue;
//                      //  }
//                        continue;
//                    }
//
//                    if((rangeError < 0.3) && (turn < 0.5)) {
//                        break;
//                    }
//
//
//
//                    // Use the speed and turn "gains" to calculate how we want the robot to move.
//                   // drive = Range.clip(rangeError * robot.SPEED_GAIN, -robot.MAX_AUTO_SPEED, robot.MAX_AUTO_SPEED);
//                 //   turn = Range.clip(headingError * robot.TURN_GAIN, -robot.MAX_AUTO_TURN, robot.MAX_AUTO_TURN);
//                   // strafe = Range.clip(-yawError * robot.STRAFE_GAIN, -robot.MAX_AUTO_STRAFE, robot.MAX_AUTO_STRAFE);
//
//                    //robot.moveRobot(drive, stra fe, turn);
//                    found = true;
//
//
//                    //robot.moveRobot(0,strafe,0);
//
//
//                    //if (Math.abs(rangeError) < 0.4) //was 0.5
//                    //    break;
//
//
//                    distanceToBoard += drive;
//
//                    if(targetReached) {
//                        break;
//                       // drive = 0;
//                       // strafe = 0;
//                    }
//
//
//
//                //}
//            }
//            else{ //not in view
//                    if(Math.abs(-gamepad1.left_stick_y) <= robot.deadband) {drive = 0;}
//                    else {drive  = -gamepad1.left_stick_y  / 2.0;}  // Reduce drive rate to 50%.
//                    if(Math.abs(-gamepad1.left_stick_x) <= robot.deadband) {strafe = 0;}
//                    else {strafe = -gamepad1.left_stick_x  / 2.0;}  // Reduce strafe rate to 50%.
//                    if(Math.abs(-gamepad1.right_stick_x) <= robot.deadband) {turn = 0;}
//                    else {turn   = -gamepad1.right_stick_x / 3.0;}  // Reduce turn rate to 33%.
//
//                    //robot.moveRobot(0,0,0); //stops robot from moving
//              //  if(found){
//                //    continue;
//                //}
//
//
//                /*
//                if (distanceToBoard < startingDistanceFromBoard) {
//                    robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Making motors run off the encoders
//                    robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    gyroDrive.driveStraight(0.2, 0.5, -90);
//                    distanceToBoard += 0.5;
//
//                }
//
//
//
//
//
//
//
//            }
//           // strafe = turn;
//            turn = 0;
//            robot.moveRobot(drive, strafe,0);
//            sleep(10);
//
//        }
//        //gyroDrive.turnToHeading(0.5,-90);
//      //  gyroDrive.driveStraight(1,robot.DESIRED_DISTANCE,-90);
//
//



    }


}
