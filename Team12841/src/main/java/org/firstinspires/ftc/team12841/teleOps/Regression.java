package org.firstinspires.ftc.team12841.teleOps;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.team12841.RobotHardware;
import org.firstinspires.ftc.team12841.configs.PanelsConfig;

@TeleOp(name = "Regression", group = "TEST")
public class Regression extends OpMode {

    /* ===================== HARDWARE ===================== */

    private RobotHardware robot;
    private Follower follower;

    /* ===================== PEDRO ===================== */

    private boolean poseReady = false;

    /* ===================== CONSTANTS ===================== */

    private static final double MAX_RPM  = 6000.0;
    private static final double RPM_STEP = 100.0;

    // Limelight rotation tuning
    private static final double LL_KP      = 0.015;
    private static final double LL_MAX_ROT = 0.6;

    /* ===================== STATE ===================== */

    private double  targetRPM      = 1000.0;
    private boolean shooterEnabled = false;

    // Button edge tracking
    private boolean dpadUpLast   = false;
    private boolean dpadDownLast = false;
    private boolean aLast        = false;

    // Turntable
    private int     ttIndex = 0;
    private boolean lbLast  = false;
    private boolean rbLast  = false;

    /* ===================== INIT ===================== */

    @Override
    public void init() {
        robot    = new RobotHardware(this);
        follower = robot.getFollower();

        // Shooter PIDF (velocity mode)
        PIDFCoefficients pidf = new PIDFCoefficients(
                0.0003, // P
                0.0,    // I
                0.0,    // D
                2.9     // F
        );

        robot.shooter.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                pidf
        );

        telemetry.addLine("Init OK — warming localization");
        telemetry.update();
    }

    /* ===================== INIT LOOP ===================== */

    @Override
    public void init_loop() {
        if (follower != null) {
            follower.update();
        }

        if (follower != null && follower.getPose() != null) {
            poseReady = true;
            telemetry.addLine("POSE READY");
        } else {
            telemetry.addLine("Warming localization...");
        }

        telemetry.update();
    }

    /* ===================== START ===================== */

    @Override
    public void start() {
        if (follower != null) {
            follower.startTeleopDrive();
            follower.update();
        }
    }

    /* ===================== LOOP ===================== */

    @Override
    public void loop() {

        /* ---------- SAFETY ---------- */

        if (follower == null || !poseReady || follower.getPose() == null) {
            telemetry.addLine("Drive unavailable");
            telemetry.update();
            return;
        }

        follower.update();

        /* ---------- DRIVE INPUTS ---------- */

        double forward = -gamepad1.right_stick_x; // rotation
        double strafe  =  gamepad1.left_stick_x;  // left / right
        double rotate  = -gamepad1.left_stick_y;  // forward / back

        /* ---------- LIMELIGHT ALIGN ---------- */

        if (gamepad1.left_trigger > 0.2) {

            robot.updateLLHeading();
            double tx = robot.getTx();

            if (tx == -999) {
                follower.setTeleOpDrive(0, 0, 0);
            } else {
                double correction =
                        clamp(-tx * LL_KP, -LL_MAX_ROT, LL_MAX_ROT);

                follower.setTeleOpDrive(0, 0, correction);
            }

        } else {
            follower.setTeleOpDrive(forward, strafe, rotate);
        }

        /* ---------- INTAKE ---------- */

        if (gamepad1.right_trigger > 0.2) {
            intakeOn();
        } else {
            intakeOff();
        }

        /* ---------- TURNTABLE ---------- */

        rotateTurntable();

        /* ---------- SHOOTER TOGGLE ---------- */

        boolean a = gamepad1.a;
        if (a && !aLast) {
            shooterEnabled = !shooterEnabled;
        }
        aLast = a;

        if (shooterEnabled) {
            robot.setShooterRPM(targetRPM);
        } else {
            robot.stopShooter();
        }

        /* ---------- RPM ADJUST ---------- */

        boolean up   = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;

        if (up && !dpadUpLast)     targetRPM += RPM_STEP;
        if (down && !dpadDownLast) targetRPM -= RPM_STEP;

        dpadUpLast   = up;
        dpadDownLast = down;

        targetRPM = clampRPM(targetRPM);

        /* ---------- TELEMETRY ---------- */

        double actualRPM =
                (robot.shooter.getVelocity() * 60.0)
                        / RobotHardware.SHOOTER_TICKS_PER_REV;

        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", actualRPM);
        telemetry.addData("TT Index", ttIndex);
        telemetry.addData("LL tx", robot.getTx());
        telemetry.update();
    }

    /* ===================== TURNTABLE ===================== */

    private void rotateTurntable() {

        boolean lb = gamepad1.left_bumper;
        boolean rb = gamepad1.right_bumper;

        if (lb && !lbLast) {
            ttIndex = (ttIndex + 2) % 3; // left wrap
            applyTTPosition();
        }

        if (rb && !rbLast) {
            ttIndex = (ttIndex + 1) % 3; // right wrap
            applyTTPosition();
        }

        lbLast = lb;
        rbLast = rb;
    }

    private void applyTTPosition() {
        switch (ttIndex) {
            case 0:
                robot.turntableServo.setPosition(PanelsConfig.TT_POS0);
                break;
            case 1:
                robot.turntableServo.setPosition(PanelsConfig.TT_POS1);
                break;
            case 2:
                robot.turntableServo.setPosition(PanelsConfig.TT_POS2);
                break;
        }
    }

    /* ===================== INTAKE ===================== */

    private void intakeOn() {
        robot.intake.setPower(1.0);
    }

    private void intakeOff() {
        robot.intake.setPower(0.0);
    }

    /* ===================== UTILS ===================== */

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    private double clampRPM(double rpm) {
        return Math.max(0.0, Math.min(MAX_RPM, rpm));
    }
}
