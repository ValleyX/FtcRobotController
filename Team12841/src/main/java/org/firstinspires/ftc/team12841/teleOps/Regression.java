package org.firstinspires.ftc.team12841.teleOps;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
    private static final double RPM_STEP = 50.0;

    // Limelight rotation tuning
    private static final double LL_KP      = 0.015;
    private static final double LL_MAX_ROT = 0.6;

    // Turntable settle tolerance
    private static final double TT_TOL = 0.02;

    /* ===================== STATE ===================== */

    private double  targetRPM      = 2800.0;
    private boolean shooterEnabled = false;

    // Button edge tracking
    private boolean dpadUpLast   = false;
    private boolean dpadDownLast = false;
    private boolean aLast        = false;
    private boolean yLast        = false;

    // Turntable
    private int     ttIndex = 0;
    private boolean lbLast  = false;
    private boolean rbLast  = false;

    private boolean servoFlick = false;

    /* ===================== Y SEQUENCE ===================== */

    private Timer yTimer;

    /* ===================== INIT ===================== */

    @Override
    public void init() {
        robot    = new RobotHardware(this);
        follower = robot.getFollower();

        resetServo();
        yTimer = new Timer();

        ttIndex = 0;
        applyTTPosition();

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
            follower.startTeleopDrive(true);
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

        double rotate  = -gamepad1.right_stick_x;
        double strafe  = -gamepad1.left_stick_x;
        double forward = -gamepad1.left_stick_y;

        /* ---------- LIMELIGHT ALIGN ---------- */

        if (gamepad1.left_trigger > 0.2) {
            robot.alignWithLimelight(-1);
        } else {
            follower.setTeleOpDrive(forward, strafe, rotate, false);
        }

        /* ---------- INTAKE ---------- */

        if (gamepad1.right_trigger > 0.2){
            robot.intake.setPower(1);
        } else if (gamepad1.b) {
            robot.intake.setPower(-1);
        } else {
            robot.intake.setPower(0);
        }

        if(gamepad1.guide)
        {
            robot.resetHeading();
        }


        /* ---------- SHOOTER TOGGLE ---------- */

        boolean a = gamepad1.a;
        if (a && !aLast) shooterEnabled = !shooterEnabled;
        aLast = a;

        targetRPM = updateRPM();
        if (shooterEnabled) robot.setShooterRPM(targetRPM);
        else robot.stopShooter();

        //if (gamepad1.dpad_up)
        //    targetRPM = targetRPM + 200;

        //if (gamepad1.dpad_down)
        //{
        //    targetRPM = targetRPM - 200;
        //}


        /* ---------- Y SEQUENCE ---------- */

        boolean y = gamepad1.y;

        if (y && !yLast)
            servoFlick = !servoFlick;

        if(servoFlick) {
            robot.leftFlick.setPosition(PanelsConfig.LEFT_SERVO_FLICK);
            if(gamepad1.x){
                robot.rightFlick.setPosition(PanelsConfig.RIGHT_SERVO_FLICK);
            } else {
                robot.rightFlick.setPosition(PanelsConfig.RIGHT_SERVO_IDLE);
            }
        } else {
            robot.leftFlick.setPosition(PanelsConfig.LEFT_SERVO_IDLE);
            robot.rightFlick.setPosition(PanelsConfig.RIGHT_SERVO_IDLE);
        }
        yLast = y;

        rotateTurntable();

        /* ---------- TELEMETRY ---------- */

        double actualRPM =
                (robot.shooter.getVelocity() * 60.0) /
                        RobotHardware.SHOOTER_TICKS_PER_REV;

        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", actualRPM);
        telemetry.addData("TT Index", ttIndex);
        telemetry.addData("LL Dis", robot.getDistance());
        telemetry.update();
    }

    private boolean ttAtTarget(double target) {
        return Math.abs(robot.turntableServo.getPosition() - target) < TT_TOL;
    }

    /* ===================== TURNTABLE ===================== */

    private void rotateTurntable() {

        boolean lb = gamepad1.left_bumper;
        boolean rb = gamepad1.right_bumper;

        if (lb && !lbLast) {
            ttIndex = (ttIndex + 3) % 4;
            applyTTPosition();
        }

        if (rb && !rbLast) {
            ttIndex = (ttIndex + 1) % 4;
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
            case 3:
                robot.turntableServo.setPosition(PanelsConfig.TT_POS3);
        }
    }

    /* ===================== UTILS ===================== */

    private double updateRPM() {
        return robot.calculateRegression(robot.getDistance());
    }

    private void resetServo() {
        robot.rightFlick.setPosition(PanelsConfig.RIGHT_SERVO_IDLE);
        robot.leftFlick.setPosition(PanelsConfig.LEFT_SERVO_IDLE);
    }
}
