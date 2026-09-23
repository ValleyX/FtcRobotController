package org.firstinspires.ftc.team12841.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.team12841.Drivers.ClimberDriver;
import org.firstinspires.ftc.team12841.Drivers.RobotHardware;


@TeleOp(name = "testTelop")
public class testTelop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor liftMotorLeft = hardwareMap.dcMotor.get("liftMotorLeft");
        DcMotor liftMotorRight = hardwareMap.dcMotor.get("liftMotorRight");
        liftMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        RobotHardware robot = new RobotHardware(this,true);
        ClimberDriver climberDriver = new ClimberDriver(robot);
        waitForStart();

        while (opModeIsActive()) {
            //liftMotorLeft.setPower(-gamepad1.left_stick_y);
          //  liftMotorRight.setPower(-gamepad1.left_stick_y);

            climberDriver.climberUp(gamepad1.left_stick_y);
           // climberDriver.climberDown(-gamepad1.left_trigger);

           /* if (gamepad1.dpad_down){
                climberDriver.climberDown(0.5);
            }*/

            robot.OpMode_.telemetry.addData("climber pos", robot.climbMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
