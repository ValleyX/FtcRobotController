package org.firstinspires.ftc.team12841.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.team12841.Drivers.ClimberDriver;
import org.firstinspires.ftc.team12841.Drivers.RobotHardware;


@TeleOp(name = "liftReset")
public class LiftReset extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        RobotHardware robot = new RobotHardware(this,true);
        ClimberDriver climberDriver = new ClimberDriver(robot);
        waitForStart();

        while (opModeIsActive()) {
            //manuel climber
            if (gamepad1.dpad_up){
                climberDriver.moveClimberNoLimit(1);
            } else if (gamepad1.dpad_down){
                climberDriver.moveClimberNoLimit(-1);
            } else {
                climberDriver.moveClimberNoLimit(0);
            }

            robot.OpMode_.telemetry.addData("climber pos", robot.climbMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}