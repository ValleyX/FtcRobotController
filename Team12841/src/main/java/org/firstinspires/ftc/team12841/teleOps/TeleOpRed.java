package org.firstinspires.ftc.team12841.teleOps;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


import org.firstinspires.ftc.team12841.RobotHardware;
import org.firstinspires.ftc.team12841.configs.PanelsConfig;

@TeleOp(name = "Red", group = "TeleOp")
public class TeleOpRed extends OpMode {

    /* ===================== HARDWARE ===================== */

    private RobotHardware robot;
    private Follower follower;

    /* ===================== CONSTANTS ===================== */

    private static final double MAX_RPM = 6000.0;
    private static final double RPM_STEP = 100.0;
    private static final double SHOOTER_TICKS_PER_REV = 28.0; // CHANGE if needed

    /* ===================== STATE ===================== */

    private double manualRPM = 0.0;
    private boolean dpadUpLast, dpadDownLast;

    private boolean shooterOn = false;
    private boolean shooterToggleLast = false;

    private int turretIndex = 0; // 0–2 looping

    /* ===================== INIT ===================== */

    @Override
    public void init() {
        robot = new RobotHardware(this);
        follower = robot.getFollower();

        if (robot.limelight != null) {
            robot.limelight.pipelineSwitch(1);
        }

        applyShooterPID();

        telemetry.addLine("TeleOp Ready");
        telemetry.update();
    }

    /* ===================== LOOP ===================== */

    @Override
    public void loop() {

        /* ===================== DRIVE ===================== */

        double forward = -gamepad1.left_stick_y;
        double strafe  =  gamepad1.left_stick_x;
        double turn    =  gamepad1.right_stick_x;

        // pad1.guide = reset PP heading
        if (gamepad1.guide) {
            robot.resetHeading();
        }

        // pad1.lt > 20% = LL align
        boolean align = gamepad1.left_trigger > 0.2;

        robot.updateLLHeading();

        if (align) {
            limelightAlignDrive(forward, strafe);
        } else {
            manualDrive(forward, strafe, turn);
        }

        /* ===================== INTAKE ===================== */

        // pad2.lt > 20% = intake 50%
        if (gamepad2.left_trigger > 0.2) {
            intakeOn();
        } else {
            intakeOff();
        }

        /* ===================== TURRET ===================== */

        // pad2.lb = left
        if (gamepad2.left_bumper) {
            turretIndex = (turretIndex + 2) % 3;
        }

        // pad2.rb = right
        if (gamepad2.right_bumper) {
            turretIndex = (turretIndex + 1) % 3;
        }

        /* ===================== SHOOTER TOGGLE ===================== */

        // pad2.b = toggle shooter
        boolean shooterToggle = gamepad2.b;
        if (shooterToggle && !shooterToggleLast) {
            shooterOn = !shooterOn;
        }
        shooterToggleLast = shooterToggle;

        /* ===================== FLICK ===================== */

        // pad2.rt = flick
        if (gamepad2.right_trigger > 0.2) {
            flick();
        } else {
            unflick();
        }

        /* ===================== RPM LOGIC ===================== */

        double distance = robot.getDistance();
        boolean targetVisible = distance != -999;

        double regressionRPM = getRPMFromRegression(distance);

        // HARD RULE: vision wins
        if (targetVisible) {
            manualRPM = regressionRPM;
        } else {
            handleManualRPMAdjust();
        }

        double targetRPM = clamp(manualRPM);

        if (shooterOn) {
            robot.setShooterRPM(targetRPM);
        } else {
            robot.stopShooter();
        }

        /* ===================== TELEMETRY ===================== */

        double ticksPerSec = robot.shooter.getVelocity();
        double actualRPM = (ticksPerSec * 60.0) / SHOOTER_TICKS_PER_REV;

        telemetry.addData("Target Visible", targetVisible);
        telemetry.addData("Distance", distance);

        telemetry.addLine();
        telemetry.addData("Regression RPM", regressionRPM);
        telemetry.addData("Manual RPM", manualRPM);
        telemetry.addData("Final RPM", targetRPM);

        telemetry.addLine();
        telemetry.addData("Actual RPM", actualRPM);
        telemetry.addData("Error", targetRPM - actualRPM);

        telemetry.addLine();
        telemetry.addData("Turret Index", turretIndex);
        telemetry.addData("tx", robot.getTx());
        telemetry.addData("Heading", robot.headingDeg());

        telemetry.update();
    }

    /* ===================== DRIVE ===================== */

    private void manualDrive(double forward, double strafe, double turn) {
        follower.setTeleOpDrive(forward, strafe, turn);
        follower.update();
    }

    private void limelightAlignDrive(double forward, double strafe) {
        double tx = robot.getTx();

        if (tx == -999) {
            manualDrive(forward, strafe, 0);
            return;
        }

        double turnCorrection = tx * PanelsConfig.LLPGAIN;
        follower.setTeleOpDrive(forward, strafe, -turnCorrection);
        follower.update();
    }

    /* ===================== RPM ADJUST (NO VISION) ===================== */

    private void handleManualRPMAdjust() {
        boolean up = gamepad2.dpad_up;
        boolean down = gamepad2.dpad_down;

        if (up && !dpadUpLast) manualRPM += RPM_STEP;
        if (down && !dpadDownLast) manualRPM -= RPM_STEP;

        manualRPM = clamp(manualRPM);

        dpadUpLast = up;
        dpadDownLast = down;
    }

    /* ===================== PID ===================== */

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

    /* ===================== REGRESSION ===================== */

    private double getRPMFromRegression(double distance) {
        if (distance <= 0 || distance == -999) return manualRPM;

        return (PanelsConfig.REGRESSION_A * distance * distance)
                + (PanelsConfig.REGRESSION_B * distance)
                + PanelsConfig.REGRESSION_C;
    }

    /* ===================== HELPERS ===================== */

    private void flick() {
        robot.leftFlick.setPosition(PanelsConfig.LEFT_SERVO_FLICK);
        robot.rightFlick.setPosition(PanelsConfig.RIGHT_SERVO_FLICK);
    }

    private void unflick() {
        robot.leftFlick.setPosition(PanelsConfig.LEFT_SERVO_IDLE);
        robot.rightFlick.setPosition(PanelsConfig.RIGHT_SERVO_IDLE);
    }

    private void intakeOn() {
        robot.intake.setPower(0.5);
    }

    private void intakeOff() {
        robot.intake.setPower(0);
    }

    /* ===================== UTILS ===================== */

    private double clamp(double rpm) {
        return Math.max(0, Math.min(MAX_RPM, rpm));
    }
}
