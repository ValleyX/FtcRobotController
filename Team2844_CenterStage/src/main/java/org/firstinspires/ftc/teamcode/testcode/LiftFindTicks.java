package org.firstinspires.ftc.team12841.testcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.team12841.Drivers.ClimberDriver;
import org.firstinspires.ftc.team12841.Drivers.LiftDrive;
import org.firstinspires.ftc.team12841.Drivers.RobotHardware;


@TeleOp(name = "liftTicks")
public class LiftFindTicks extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        RobotHardware robot = new RobotHardware(this,true);
        LiftDrive liftDriver = new LiftDrive(robot);
        boolean isBucketClosed = true;
        waitForStart();

        while (opModeIsActive()) {
            //manuel climber
            if (gamepad2.dpad_up){
                liftDriver.moveLift(1);
            } else if (gamepad2.dpad_down){
                liftDriver.moveLift(-1);
            } else {
                liftDriver.moveLift(0);
            }

            if (gamepad2.right_trigger > .5) {
                if (isBucketClosed) {
                    //robot.bucketServo.setPosition(robot.BUCKET_MIDDLE);
                   // sleep(50);
                    robot.bucketServo.setPosition(robot.BUCKET_OPEN-.05);
                    sleep(100);
                    isBucketClosed = false;
                }
            } else {
                robot.bucketServo.setPosition(robot.BUCKET_CLOSED);
                isBucketClosed = true;
            }

            robot.OpMode_.telemetry.addData("lift pos right", robot.liftMotorRight.getCurrentPosition());
            robot.OpMode_.telemetry.addData("lift pos left", robot.liftMotorLeft.getCurrentPosition());
            telemetry.update();
        }
    }
}