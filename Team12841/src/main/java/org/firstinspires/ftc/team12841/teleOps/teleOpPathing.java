package org.firstinspires.ftc.team12841.teleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.team12841.RobotHardware;

@TeleOp(name = "TeleOp Main", group = "opModes")
public class teleOpPathing extends OpMode {

    private RobotHardware robot;
    private boolean shooterActive = false;
    private boolean CONTROLPressedPrev = false;

    private boolean babyMode = false;
    private boolean babyTogglePrev = false;
    private final double BABY_MODE_SCALE = 0.4;

    // --- Servo preset positions ---
    private final double SHOOTER_IDLE = 0.15; // CHANGE THIS
    private final double SHOOTER_FIRE = 0.85; // CHANGE THIS
    private final double TURNTABLE_POS1 = 0.2; // CHANGE THIS
    private final double TURNTABLE_POS2 = 0.8; // CHANGE THIS
    private final double TURNTABLE_POS3 = 0.0; // CHANGE THIS

    @Override
    public void init() {
        robot = new RobotHardware(this);
        telemetry.addLine("TeleOp initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- Toggle baby mode (example: right bumper) ---
        if (gamepad1.right_bumper && !babyTogglePrev) {
            babyMode = !babyMode;
        }
        babyTogglePrev = gamepad1.right_bumper;

        // Read joystick input
        double x = gamepad1.right_stick_x;
        double y = -gamepad1.right_stick_y;
        double rot = gamepad1.left_stick_x;

        // Apply baby mode scaling
        double speedScale = babyMode ? BABY_MODE_SCALE : 1.0;
        x *= speedScale;
        y *= speedScale;
        rot *= speedScale;

        // --- Drive control ---
        if (robot.getFollower() != null) {
            robot.getFollower().setTeleOpDrive(y, x, rot, false); // field-centric
        } else {
            driveManual(y, x, rot);
        }

        // --- Shooter motor toggle (example: assign to gamepad2.a) ---
        if (gamepad2.CONTROL && !CONTROLPressedPrev) shooterActive = !shooterActive;
        CONTROLPressedPrev = gamepad2.CONTROL;
        setShooterPower(shooterActive ? 1.0 : 0.0);

        // --- Shooter servo control (example: gamepad2.right_bumper fires) ---
        if (gamepad2.CONTROL) setShooterServo(SHOOTER_FIRE);
        else setShooterServo(SHOOTER_IDLE);

        // --- Turntable servo presets (example: dpad) ---
        if (gamepad2.CONTROL) setTurntableServo(TURNTABLE_POS1);
        else if (gamepad2.CONTROL) setTurntableServo(TURNTABLE_POS2);
        else if (gamepad2.CONTROL) setTurntableServo(TURNTABLE_POS3);

        // --- Telemetry ---
        telemetry.addData("Shooter Active", shooterActive);
        telemetry.addData("Baby Mode", babyMode ? "ON" : "OFF");
        telemetry.update();
    }

    // ========== CONTROL METHODS ==========

    private void setShooterPower(double power) {
        if (robot.shooterMotor != null) robot.shooterMotor.setPower(power);
    }

    private void setShooterServo(double position) {
        if (robot.shooterServo != null) robot.shooterServo.setPosition(position);
    }

    private void setTurntableServo(double position) {
        if (robot.turntableServo != null) robot.turntableServo.setPosition(position);
    }

    private void driveManual(double y, double x, double rot) {
        double fl = y + x + rot;
        double fr = y - x - rot;
        double bl = y - x + rot;
        double br = y + x - rot;

        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

        robot.lfMotor.setPower(fl / max);
        robot.rfMotor.setPower(fr / max);
        robot.lbMotor.setPower(bl / max);
        robot.rbMotor.setPower(br / max);
    }
}
