package org.firstinspires.ftc.team12841.teleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.team12841.RobotHardware;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "TeleOp Main", group = "opModes")
public class teleOp extends OpMode {

    private RobotHardware robot;
    private double headingOffset = 0;
    private boolean babyMode = false;

    @Override
    public void init() {
        robot = new RobotHardware(this);
        telemetry.addLine("Initialized");
    }

    @Override
    public void loop() {

        // Get inputs from controller
        double x = -gamepad1.left_stick_x;   // Strafe (invert if needed)
        double y = -gamepad1.left_stick_y;   // Forward/backward
        double rotation = gamepad1.right_stick_x;  // Rotation

        // Dpad controls
        // if (gamepad1.a) headingOffset = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); // Reset heading
        // if (gamepad1.dpad_down) babyMode = true;
        // if (gamepad1.dpad_up) babyMode = false;

        // The IMU heading
        double heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - headingOffset;

        // The field centric stuff
        double rotX = x * Math.cos(heading) - y * Math.sin(heading);
        double rotY = x * Math.sin(heading) + y * Math.cos(heading);

        // Baby Mode
        double speedScale = babyMode ? 0.4 : 1.0;
        rotX *= speedScale;
        rotY *= speedScale;
        rotation *= speedScale;

        //  OUTPUT
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

        // FEEDBACK / TELEMETRY

        telemetry.addData("Heading (°)", Math.toDegrees(heading));
        telemetry.addData("Offset (°)", Math.toDegrees(headingOffset));
        telemetry.addData("Slow Mode", babyMode);
        telemetry.addData("Powers", "FL: %.2f | FR: %.2f | BL: %.2f | BR: %.2f",
                frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        telemetry.update();
    }
}
