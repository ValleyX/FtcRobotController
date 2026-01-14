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

    private RobotHardware robot;
    private Follower follower;

    /* ===================== CONSTANTS ===================== */

    private static final double MAX_RPM = 6000.0;
    private static final double RPM_STEP = 100.0;

    /* ===================== STATE ===================== */

    private double targetRPM = 3000.0;
    private boolean shooterEnabled = false;

    private boolean dpadUpLast, dpadDownLast;
    private boolean aLast;

    @Override
    public void init() {
        robot = new RobotHardware(this);
        follower = robot.getFollower();

        applyShooterPID();

        telemetry.addLine("Regression TeleOp Loaded");
        telemetry.update();
    }

    @Override
    public void loop() {

        /* ===================== DRIVE (PEDRO) ===================== */

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );
        follower.update();

        /* ===================== LIMELIGHT ALIGN (LT) ===================== */

        if (gamepad2.left_trigger > 0.2) {
            robot.updateLLHeading();
            robot.alignWithLimelight(gamepad2.left_trigger);
        } else {
            robot.addAlign(0, 0, 0, 0);
        }

        /* ===================== FLICK (RT) ===================== */

        if (gamepad2.right_trigger > 0.2) {
            flick(); // PLACEHOLDER
        }

        /* ===================== TT ROTATION (BUMPERS) ===================== */

        if (gamepad2.left_bumper) {
            rotateTTLeft(); // PLACEHOLDER
        } else if (gamepad2.right_bumper) {
            rotateTTRight(); // PLACEHOLDER
        }

        /* ===================== SHOOTER TOGGLE (A) ===================== */

        boolean a = gamepad2.a;
        if (a && !aLast) {
            shooterEnabled = !shooterEnabled;
        }
        aLast = a;

        if (shooterEnabled) {
            robot.setShooterRPM(targetRPM);
        } else {
            robot.stopShooter();
        }

        /* ===================== RPM TARGET ADJUST ===================== */

        boolean up = gamepad2.dpad_up;
        boolean down = gamepad2.dpad_down;

        if (up && !dpadUpLast) targetRPM += RPM_STEP;
        if (down && !dpadDownLast) targetRPM -= RPM_STEP;

        dpadUpLast = up;
        dpadDownLast = down;

        targetRPM = clamp(targetRPM);

        /* ===================== PID RE-APPLY ===================== */

        if (gamepad2.x) {
            applyShooterPID();
        }

        /* ===================== TELEMETRY ===================== */

        double ticksPerSec = robot.shooter.getVelocity();
        double actualRPM =
                (ticksPerSec * 60.0) / RobotHardware.SHOOTER_TICKS_PER_REV;

        double rpmError = targetRPM - actualRPM;

        telemetry.addData("Shooter Enabled", shooterEnabled);
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", actualRPM);
        telemetry.addData("RPM Error", rpmError);

        telemetry.addLine();
        telemetry.addData("LL Distance", robot.getDistance());
        telemetry.addData("LL tx", robot.getTx());

        telemetry.addLine();
        telemetry.addData("P", PanelsConfig.SHOOTER_P);
        telemetry.addData("I", PanelsConfig.SHOOTER_I);
        telemetry.addData("D", PanelsConfig.SHOOTER_D);
        telemetry.addData("F", PanelsConfig.SHOOTER_F);

        telemetry.addData(
                "At Speed",
                Math.abs(rpmError) < PanelsConfig.SHOOTER_READY_RPM_ERROR
        );

        telemetry.addLine();
        telemetry.addLine("LT=Align | RT=Flick | A=Shooter");
        telemetry.addLine("LB/RB=TT | Dpad=RPM | X=Apply PID");

        telemetry.update();
    }

    /* ===================== PID APPLY ===================== */

    private void applyShooterPID() {
        PIDFCoefficients pidf = new PIDFCoefficients(
                PanelsConfig.SHOOTER_P,
                PanelsConfig.SHOOTER_I,
                PanelsConfig.SHOOTER_D,
                PanelsConfig.SHOOTER_F
        );

        robot.shooter.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                pidf
        );
    }

    /* ===================== PLACEHOLDERS ===================== */

    private void flick() {
        // TODO: pulse servo or motor
    }

    private void rotateTTLeft() {
        // TODO: rotate turntable left
    }

    private void rotateTTRight() {
        // TODO: rotate turntable right
    }

    /* ===================== UTILS ===================== */

    private double clamp(double v) {
        return Math.max(0, Math.min(MAX_RPM, v));
    }
}
