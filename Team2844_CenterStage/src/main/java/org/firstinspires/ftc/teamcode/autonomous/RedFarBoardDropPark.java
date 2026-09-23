package org.firstinspires.ftc.team12841.autonomous;

/*
Written by Benjamin Ettinger
Made: 11/7/23
Blue Near Board Autonomous, detect the right spike, drop pixel, go park
 */


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.Drivers.AprilTag;
import org.firstinspires.ftc.team12841.Drivers.GyroDrive;
import org.firstinspires.ftc.team12841.Drivers.IntakeDriver;
import org.firstinspires.ftc.team12841.Drivers.LiftDrive;
import org.firstinspires.ftc.team12841.Drivers.RobotHardware;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Disabled
@Autonomous(name="RedNearDropPark")
public class RedFarBoardDropPark extends LinearOpMode {

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
        GyroDrive gyroDrive = new GyroDrive(robot);
        LiftDrive liftDrive = new LiftDrive(robot);
        IntakeDriver intakeDriver = new IntakeDriver(robot);
        RobotHardware.CenterStagePipeline.DetectionPosition position = RobotHardware.CenterStagePipeline.DetectionPosition.Left; // position robot detects
        AprilTagDetection desiredTag = null;
        AprilTag aprilTag;



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


        //DETECT SPIKE POSITION
        //DETECTS RIGHT
        if (position == robot.pipeline.position.Right) {

            //drive forward
            gyroDrive.driveStraight(0.5, 30, 0);

            //turn
            gyroDrive.turnToHeading(0.5, 90);

            //drop the pixel on mark
            intakeDriver.intakeOn(true, -0.75);
            sleep(500);
            intakeDriver.intakeOn(false, 0);


            //turn again
            gyroDrive.turnToHeading(0.5, -90);


            //go to park
            gyroDrive.driveStraight(0.5, 25, -90);
            gyroDrive.turnToHeading(0.5, -180);
            gyroDrive.driveStraight(0.5, 25, -180);
            gyroDrive.turnToHeading(0.5, -90);
            gyroDrive.driveStraight(0.5, 25, -90);

        }
        //DECTECTS MIDDLE
        else if (position == robot.pipeline.position.Middle) {

            //drive forward
            gyroDrive.driveStraight(0.5, 30, 0);

            //drop the pixel on mark
            intakeDriver.intakeOn(true, -0.75);
            sleep(500);
            intakeDriver.intakeOn(false, 0);


            //turn
            gyroDrive.turnToHeading(0.5, -90);

            //go to park
            gyroDrive.driveStraight(0.5, 25, -90);
            gyroDrive.turnToHeading(0.5, -180);
            gyroDrive.driveStraight(0.5, 25, -180);
            gyroDrive.turnToHeading(0.5, -90);
            gyroDrive.driveStraight(0.5, 25, -90);

        }
        //DETECTS LEFT (DEFAULT CONDITION)
        else {

            //drive forward
            gyroDrive.driveStraight(0.5, 30, 0);

            //turn
            gyroDrive.turnToHeading(0.5, -90);

            //drop the pixel on mark
            intakeDriver.intakeOn(true, -0.75);
            sleep(500);
            intakeDriver.intakeOn(false, 0);

            //go to park
            gyroDrive.driveStraight(0.5, 25, 90);
            gyroDrive.turnToHeading(0.5, 180);
            gyroDrive.driveStraight(0.5, 25, 180);
            gyroDrive.turnToHeading(0.5, 90);
            gyroDrive.driveStraight(0.5, 25, 90);
          //  gyroDrive.turnToHeading(0.5, 0);
        }
        



    } //runOpMode end bracket





} //BlueNearBoardDropPark end bracket
