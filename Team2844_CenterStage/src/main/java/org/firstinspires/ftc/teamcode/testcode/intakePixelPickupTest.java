package org.firstinspires.ftc.team12841.testcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team12841.Drivers.AprilTag;
import org.firstinspires.ftc.team12841.Drivers.ClimberDriver;
import org.firstinspires.ftc.team12841.Drivers.GyroDrive;
import org.firstinspires.ftc.team12841.Drivers.IntakeDriver;
import org.firstinspires.ftc.team12841.Drivers.LiftDrive;
import org.firstinspires.ftc.team12841.Drivers.RobotHardware;

import org.firstinspires.ftc.team12841.Drivers.OdometryDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

//robot must pe put in the same place to relatively same position per game to be consistant
//Position distance from corner near board is about 48 inches

@Autonomous(name="intaxkepixeltest")
public class intakePixelPickupTest extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {

        //Using test hardware for now because test hardware has properties needed for practice
        RobotHardware robot = new RobotHardware(this,true); // use true if it is blueside

        GyroDrive gyroDrive = new GyroDrive(robot);//sets up Drives
        LiftDrive liftDrive = new LiftDrive(robot);
        IntakeDriver intakeDrive = new IntakeDriver(robot);


        RobotHardware.CenterStagePipeline.DetectionPosition position = RobotHardware.CenterStagePipeline.DetectionPosition.Left; // position robot detects
        AprilTagDetection desiredTag = null;
        AprilTag aprilTag;

        //robot.calibrateNavX();

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


        intakeDrive.intakeOn(true,1);
        gyroDrive.driveStraight(.3,-22,0);
        sleep(1000);
        gyroDrive.driveStraight(.5,22,0);
        intakeDrive.intakeOn(false,0);

       // gyroDrive.turnToHeading(.5,90);



    }


}