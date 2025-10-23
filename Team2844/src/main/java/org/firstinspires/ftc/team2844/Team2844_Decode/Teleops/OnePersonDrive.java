package org.firstinspires.ftc.team2844.Team2844_Decode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team2844.Team2844_Decode.Hardwares.RobotHardware;

@TeleOp(name = "OnePersonDrive")
public class OnePersonDrive extends LinearOpMode {
    RobotHardware robotHardware;
    LinearOpMode OpMode_;


    //feild centric vars
    double x;
    double y;
    double rx;
    double botHeading;



    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware = new RobotHardware(this);
        OpMode_ = this;

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

            robotHardware.powerMotors(frontLeftPower, backLeftPower, backRightPower, frontRightPower);
        }
    }
}
