package org.firstinspires.ftc.team12841.testcode;

/*
Written by Benjamin Ettinger
Made: 11/7/23
Blue Near Board Autonomous, detect the right spike, drop pixel, go park
 */


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.Drivers.AprilTag;
import org.firstinspires.ftc.team12841.Drivers.GyroDrive;
import org.firstinspires.ftc.team12841.Drivers.IntakeDriver;
import org.firstinspires.ftc.team12841.Drivers.LiftDrive;
import org.firstinspires.ftc.team12841.Drivers.RobotHardware;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name="RedFarBoard")
public class RedFarBoardTest extends LinearOpMode {

    /*
    OUTLINE
        Detect Spike Position
        Drop Pixel at corresponding location
        Drive to the Parking Position
    */



    @Override
    public void runOpMode() throws InterruptedException {




        //gets drivers
        RobotHardware robot = new RobotHardware(this, false); //checkBlue is true to only find
        GyroDrive gyroDrive = new GyroDrive(robot);
        LiftDrive liftDrive = new LiftDrive(robot);
        IntakeDriver intakeDriver = new IntakeDriver(robot);
        RobotHardware.CenterStagePipeline.DetectionPosition position = RobotHardware.CenterStagePipeline.DetectionPosition.Left; // position robot detects
        AprilTagDetection desiredTag = null;
        AprilTag aprilTag;

        robot.pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
        robot.blinkinLedDriver.setPattern(robot.pattern);



        while(opModeInInit()) {
            //to tell user what values the camera sees
            telemetry.addData("r value", robot.pipeline.avgR);
            telemetry.addData("b value", robot.pipeline.avgB);
            telemetry.addData("r2 value", robot.pipeline.avg2R);
            telemetry.addData("b2 value", robot.pipeline.avg2B);
            telemetry.addData("r3 value", robot.pipeline.avg3R);
            telemetry.addData("b3 value", robot.pipeline.avg3B);

            telemetry.addData("teamProp position", robot.pipeline.position);

            telemetry.update();

            position = robot.pipeline.position;
        }



        int AprilTagID = 2;
        double drive = 0;
        double turn = 0;
        double strafe = 0;
        double distanceToBoard = 0;
        double startingDistanceFromBoard = 49;


        waitForStart(); //waits
        //robot.imu.resetYaw(); //reset IMU
        robot.pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        robot.blinkinLedDriver.setPattern(robot.pattern);


        //DETECT SPIKE POSITION
        //DETECTS LEFT
        if (position == robot.pipeline.position.Left) {

            //drive straight
            gyroDrive.driveStraight(0.25, 20, 0);

            //turn to the marker
            gyroDrive.turnToHeading(0.25, -45);

            //drive forward
            gyroDrive.driveStraight(0.25, 15, -45);

            //turn straight
            gyroDrive.turnToHeading(0.5, 0);

            //go forward a bit
            gyroDrive.driveStraight(0.5, 12, 0);

            //drop pixel
            intakeDriver.intakeOn(true, -0.2);
            sleep(2450);
            intakeDriver.intakeOn(false, 0);

            //go forward a bit
            gyroDrive.driveStraight(0.5, 8, 0);

            /*//turn to park
            gyroDrive.turnToHeading(0.5, 90);

            //book it over to board
            gyroDrive.driveStraight(0.7, 80, 95);

            //turn
            gyroDrive.turnToHeading(.5,180);

            //drive to where we want to place pixel
            gyroDrive.driveStraight(.5,19,180);

            //turn
            gyroDrive.turnToHeading(.5,90);

            //drive to board
            gyroDrive.driveStraight(.25,29,90);

            ////placePixel
            liftDrive.liftToHeight(8,1,.1,1000,true);
            robot.bucketServo.setPosition(robot.BUCKET_OPEN);
            sleep(1000);//move srevo on bucket
            robot.bucketServo.setPosition(robot.BUCKET_CLOSED);
            sleep(1000);
            liftDrive.liftReset();

            //back away
            gyroDrive.driveStraight(.25,-7,90);

            //turn to face correctly
            gyroDrive.turnToHeading(.5,0);

            //drive out of the way to board
            gyroDrive.driveStraight(.5,11,0);

            //double check we are centerd
            gyroDrive.turnToHeading(.5,0);*/

        }
        //DECTECTS MIDDLE
        else if (position == robot.pipeline.position.Middle) {
//go forward a lot
            gyroDrive.driveStraight(.3, 48.5, 0);

            //drop pixel
            intakeDriver.intakeOn(true, 0.4);
            sleep(2300);
            intakeDriver.intakeOn(false, 0);

            //move forward a bit
            gyroDrive.driveStraight(0.5, 2, 0);

            //turn
            gyroDrive.waitForAllMotors = true;
            gyroDrive.turnToHeading(0.3, 90);
            gyroDrive.waitForAllMotors = false;
            gyroDrive.turnToHeading(0.3, 90);

            sleep(5000);

            //book it over to board
            gyroDrive.driveStraight(0.2, 40, 90);
            sleep(500);
            gyroDrive.driveStraight(0.4, 40, 90);

            //turn
            gyroDrive.turnToHeading(.5, 180);

            //drive to where we want to place pixel
            gyroDrive.driveStraight(.5, 24.5, 180);

            //turn
            gyroDrive.turnToHeading(.5, 90);

            //drive to board
            gyroDrive.driveStraight(.25, 15, 90);

            ////placePixel
            liftDrive.liftToHeight(8, 1, .1, 500, true);
            robot.bucketServo.setPosition(robot.BUCKET_OPEN);
            sleep(1000);//move srevo on bucket
            robot.bucketServo.setPosition(robot.BUCKET_CLOSED);
            sleep(1000);
            liftDrive.liftReset();

            //back away
            gyroDrive.driveStraight(.25, -5, 90);

            //turn to face correctly
            gyroDrive.turnToHeading(.5, 0);

            //drive out of the way to board
            gyroDrive.driveStraight(.5, 17, 0);

            //double check we are centerd
            gyroDrive.turnToHeading(.5, 0);

        }
        //DETECTS RIGHT (DEFAULT CONDITION)
        else {

            //drive straight
            gyroDrive.driveStraight(0.25, 35, 0);

            //turn intake to marker
            gyroDrive.turnToHeading(0.25, -90);

            //back up to take the dump
            //gyroDrive.driveStraight(0.25, , 90);

            //drop pixel
            intakeDriver.intakeOn(true, 0.3);
            sleep(2250);
            intakeDriver.intakeOn(false, 0);

            //go forward a bit
            gyroDrive.driveStraight(0.5, 3, 0);

            //turn
            gyroDrive.turnToHeading(0.5, 0);

            //go straight unlike david
            gyroDrive.driveStraight(0.5, 18, 0);

            //turn to park
            gyroDrive.waitForAllMotors = true;
            gyroDrive.turnToHeading(0.3, 90);
            gyroDrive.waitForAllMotors = false;

            //book it over to board
            gyroDrive.driveStraight(0.15, 40, 90);
            sleep(500);
            gyroDrive.driveStraight(0.2, 40, 90);

            //turn
            gyroDrive.turnToHeading(.6, 180);

            //drive to where we want to place pixel
            gyroDrive.driveStraight(.5, 30, 180);

            //turn
            gyroDrive.turnToHeading(.6, 90);

            //drive to board
            gyroDrive.driveStraight(.25, 18, 90);

            liftDrive.liftToHeight(8,1,.1,1000,true);
            robot.bucketServo.setPosition(robot.BUCKET_OPEN - .03);
            sleep(1000);//move srevo on bucket
            robot.bucketServo.setPosition(robot.BUCKET_CLOSED);
            sleep(1000);
            liftDrive.liftReset();
            sleep(1000); //-------------- sleep to put lift pixel here

            //back away
            gyroDrive.driveStraight(.25, -7, 90);

            //turn to face correctly
            gyroDrive.turnToHeading(.5, 0);

            //drive out of the way to board
            gyroDrive.driveStraight(.5, 15, 0);

            //double check we are centerd
            gyroDrive.turnToHeading(.5, 0);


        }




    } //runOpMode end bracket





} //BlueNearBoardDropPark end bracket
