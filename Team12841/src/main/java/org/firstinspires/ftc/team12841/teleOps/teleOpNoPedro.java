package org.firstinspires.ftc.team12841.teleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.team12841.RobotHardware;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "TeleOp No Pathing", group = "opModes")
public class teleOpNoPedro extends OpMode {

    private RobotHardware robot;
    private double headingOffset = 0;
    private boolean babyMode = false;
    private final double BABY_MODE_VAL = 0.4;

    @Override
    public void init() {
        robot = new RobotHardware(this);
        telemetry.addLine("Initialized");
        robot.imu.resetYaw();
    }

    @Override
    public void loop() {

        // Get inputs from controller
        double x = gamepad1.left_stick_x;   // Strafe (invert if needed)
        double y = -gamepad1.left_stick_y;   // Forward/backward
        double rotation = gamepad1.right_stick_x;  // Rotation

        // Controls
        // if (gamepad1.a) headingOffset = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); // Reset heading
        if (gamepad1.right_bumper) babyMode = true;
        if (gamepad1.b) babyMode = false;

        // The IMU heading
        double heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - headingOffset;

        // The field centric stuff
        double rotX = x * Math.cos(heading) - y * Math.sin(heading);
        double rotY = x * Math.sin(heading) + y * Math.cos(heading);

        // Baby Mode
        double speedScale = babyMode ? BABY_MODE_VAL : 1.0; // If BabyMode, SpeedScale = 0.4, else, SpeedScale = 1.0
        rotX *= speedScale;
        rotY *= speedScale;
        rotation *= speedScale;

        //  Motor Outputs
        double frontLeftPower  = rotY + rotX + rotation;
        double frontRightPower = rotY - rotX - rotation;
        double backLeftPower   = rotY - rotX + rotation;
        double backRightPower  = rotY + rotX - rotation;

        // Normalize power levels
        double maxPower = Math.max(1.0, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        frontLeftPower  /= maxPower;
        frontRightPower /= maxPower;
        backLeftPower   /= maxPower;
        backRightPower  /= maxPower;

        robot.lfMotor.setPower(frontLeftPower);
        robot.rfMotor.setPower(frontRightPower);
        robot.lbMotor.setPower(backLeftPower);
        robot.rbMotor.setPower(backRightPower);

        // Telemetry
        telemetry.addData("Heading (°)", Math.toDegrees(heading));
        telemetry.addData("Offset (°)", Math.toDegrees(headingOffset));
        telemetry.addData("Baby Mode", babyMode);
        telemetry.addData("Powers", "FL: %.2f | FR: %.2f | BL: %.2f | BR: %.2f",
                frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        telemetry.update();
    }
}
