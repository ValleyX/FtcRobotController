package org.firstinspires.ftc.team12841.teleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.team12841.RobotHardware;

@TeleOp(name="TeleOp", group="Linear Opmode")
public class teleOp extends OpMode {

    private RobotHardware robot;

    @Override
    public void init() {
        // Initialize hardware
        robot = new RobotHardware(this);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Gamepad inputs
        double drive  = -gamepad1.left_stick_y;   // Forward/back
        double strafe =  gamepad1.left_stick_x;   // Left/right
        double turn   =  gamepad1.right_stick_x;  // Rotation

        // Mecanum drive math
        double rfPower = drive - strafe - turn;
        double rbPower = drive + strafe - turn;
        double lfPower = drive - strafe + turn;
        double lbPower = drive + strafe + turn;

        // Normalize powers so no value exceeds 1.0
        double max = Math.max(1.0, Math.max(Math.abs(rfPower),
                Math.max(Math.abs(rbPower),
                        Math.max(Math.abs(lfPower), Math.abs(lbPower)))));
        rfPower /= max;
        rbPower /= max;
        lfPower /= max;
        lbPower /= max;

        // Apply power to motors
        robot.rfMotor.setPower(rfPower);
        robot.rbMotor.setPower(rbPower);
        robot.lfMotor.setPower(lfPower);
        robot.lbMotor.setPower(lbPower);

        // Telemetry for debugging
        telemetry.addData("Drive", "fwd=%.2f strafe=%.2f turn=%.2f", drive, strafe, turn);
        telemetry.addData("Motors", "lf=%.2f lb=%.2f rf=%.2f rb=%.2f", lfPower, lbPower, rfPower, rbPower);
        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop all motors when OpMode ends
        robot.rfMotor.setPower(0);
        robot.rbMotor.setPower(0);
        robot.lfMotor.setPower(0);
        robot.lbMotor.setPower(0);
    }
}