package org.firstinspires.ftc.teamcode.autonomous;

/*
Written by Benjamin Ettinger
Made: 11/7/23
Blue Near Board Autonomous, detect the right spike, drop pixel, go park
 */


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Drivers.AprilTag;
import org.firstinspires.ftc.teamcode.Drivers.GyroDrive;
import org.firstinspires.ftc.teamcode.Drivers.IntakeDriver;
import org.firstinspires.ftc.teamcode.Drivers.LiftDrive;
import org.firstinspires.ftc.teamcode.Drivers.RobotHardware;
import org.firstinspires.ftc.teamcode.testcode.GyroDriveTest;
import org.firstinspires.ftc.teamcode.testcode.RobotHardwareTestVersion;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name="BlueFarDropPark")
public class BlueFarDropPark extends LinearOpMode {

    /*
    OUTLINE
        Detect Spike Position
        Drop Pixel at corresponding location
        Drive to the Parking Position
    */



    @Override
    public void runOpMode() throws InterruptedException {




        //gets drivers
        RobotHardware robot = new RobotHardware(this, true); //checkBlue is true to only find
        GyroDrive gyroDrive = new GyroDrive(robot);//sets up Drives        LiftDrive liftDrive = new LiftDrive(robot);
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
        //obot.imu.resetYaw(); //reset IMU
        robot.pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        robot.blinkinLedDriver.setPattern(robot.pattern);


        //DETECT SPIKE POSITION
        //DETECTS LEFT
        if (position == robot.pipeline.position.Left) {


            //drive straight
            gyroDrive.driveStraight(0.25, 35, 0);

            //turn intake to marker
            gyroDrive.turnToHeading(0.25, 90);

            //back up to take the dump
            gyroDrive.driveStraight(0.25, -2, 90);

            //drop pixel
            intakeDriver.intakeOn(true, .3);
            sleep(2250);
            intakeDriver.intakeOn(false, 0);

            //go forward a bit
            gyroDrive.driveStraight(0.5, 5, 0);

            //turn
            gyroDrive.turnToHeading(0.5, 0);

            //go straight
            gyroDrive.driveStraight(0.5, 18, 0);

            //turn to park
            gyroDrive.turnToHeading(0.5, -90);

            //go park
            gyroDrive.driveStraight(0.5, 105, -92);

        }
        //DECTECTS MIDDLE
        else if (position == robot.pipeline.position.Middle) {

            //go forward a lot
            gyroDrive.driveStraight(0.25, 50, 0);

            //drop pixel
            intakeDriver.intakeOn(true, 0.3);
            sleep(2350);
            intakeDriver.intakeOn(false, 0);

            //move forward a bit
            gyroDrive.driveStraight(0.5, 3, 0);

            //turn
            gyroDrive.turnToHeading(0.5, -90);

            //book it over to park
            gyroDrive.driveStraight(0.5, 100, -92);

        }
        //DETECTS RIGHT (DEFAULT CONDITION)
        else {

            //drive straight
            gyroDrive.driveStraight(0.25, 20, 0);

            //turn to the marker
            gyroDrive.turnToHeading(0.25, 45);

            //drive forward
            gyroDrive.driveStraight(0.25, 15, 45);

            //turn straight
            gyroDrive.turnToHeading(0.5, 0);

            //go forward a bit
            gyroDrive.driveStraight(0.5, 10, 0);

            //drop pixel
            intakeDriver.intakeOn(true, .3);
            sleep(2250);
            intakeDriver.intakeOn(false, 0);

            //go forward a bit
            gyroDrive.driveStraight(0.5, 10, 0);

            //turn to park
            gyroDrive.turnToHeading(0.5, -90);

            //go park
            gyroDrive.driveStraight(0.5, 110, -92);


        }




    } //runOpMode end bracket





} //BlueNearBoardDropPark end bracket
