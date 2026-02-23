package org.firstinspires.ftc.team12841.teleOps;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team12841.RobotHardware;
import org.firstinspires.ftc.team12841.configs.PanelsConfig;

@Disabled
@TeleOp(name = "BASE")
public class TeleOpBase extends OpMode {

    /* ===================== HARDWARE ===================== */
    private RobotHardware robot;
    private Follower follower;

    /* ===================== PEDRO ===================== */
    private boolean poseReady = false;

    /* ===================== CONSTANTS ===================== */
    private static final double MAX_RPM = 6000.0;
    private static final double RPM_STEP = 50.0;

    // Limelight rotation tuning
    private static final double LL_KP = 0.04;
    private static final double LL_MAX_ROT = 0.8;

    /* ===================== STATE ===================== */
    private double targetRPM = 2800.0;
    private boolean shooterEnabled = true;

    public boolean babyMode = false;
    public int pipeline = 0;

    private Timer timer;
    private Timer flash;
    boolean flashActive = false;

    /* ===================== INIT ===================== */
    @Override
    public void init() {
        robot = new RobotHardware(this);
        follower = robot.getFollower();

        telemetry.addLine("Init OK — warming localization");
        telemetry.update();

        robot.limelight.pipelineSwitch(pipeline);
        timer = new Timer();
        flash = new Timer();
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
            timer.resetTimer();
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
        double rotate = -gamepad1.right_stick_x;
        double strafe = -gamepad1.left_stick_x;
        double forward = -gamepad1.left_stick_y;

        if (gamepad1.startWasPressed()) {
            babyMode = !babyMode;
        }

        /* ---------- LIMELIGHT ALIGN ---------- */
        if (gamepad1.left_trigger > 0.2) {
            follower.setTeleOpDrive(0, 0, 0, false); // freeze Pedro
            robot.alignWithLimelight(-1);
        } else if (babyMode) {
            follower.setTeleOpDrive(
                    forward * PanelsConfig.BABY,
                    strafe * PanelsConfig.BABY,
                    rotate * PanelsConfig.BABY,
                    false
            );
        } else {
            follower.setTeleOpDrive(forward, strafe, rotate, false);
        }

        /* ---------- INTAKE ---------- */
        if (gamepad1.right_trigger > 0.2 && !robot.isBroken()) {
            robot.intake.setPower(1);
            robot.flick.setPower(-1);
        } else if (gamepad1.right_trigger > 0.2 && robot.isBroken()) {
            robot.intake.setPower(1);
        } else if (gamepad1.b) {
            robot.intake.setPower(-1);
        } else if (gamepad1.x) {
            robot.flick.setPower(-1);
        } else if (gamepad1.y) {
            robot.flick.setPower(1);
        } else {
            robot.intake.setPower(0);
            robot.flick.setPower(0);
        }

        if (gamepad1.guide) {
            robot.resetHeading();
        }

        /* ---------- SHOOTER ---------- */
        targetRPM = updateRPM();

        if (shooterEnabled) {
            robot.setShooterRPM(targetRPM);
        } else {
            robot.stopShooter();
        }

        /*if(gamepad1.dpadUpWasPressed())
        {
            targetRPM += 50;
        } else if(gamepad1.dpadDownWasPressed()) {
            targetRPM -= 50;
        }*/

        /* ---------- FLICK ---------- */
        if (gamepad1.x) {
            robot.flick.setPower(-1);
        } else if (gamepad1.y) {
            robot.flick.setPower(1);
        } else {
            robot.flick.setPower(0);
        }

        /* ---------- TELEMETRY ---------- */
        double actualRPM =
                (robot.shooter.getVelocity() * 60.0)
                        / RobotHardware.SHOOTER_TICKS_PER_REV;

        double t = timer.getElapsedTimeSeconds();

        if (t >= 100) {
            // RED FLASHING (100s → end)
            if (!flashActive) {
                flash.resetTimer();
                flashActive = true;
            }

            double ft = flash.getElapsedTimeSeconds();

            if (ft < 0.333) {
                robot.setGoBildaLight(0.27); // Red ON
            } else if (ft < 0.667) {
                robot.setGoBildaLight(0.0); // OFF
            } else if (ft < 1.0) {
                robot.setGoBildaLight(0.27); // Red ON
            } else {
                flash.resetTimer(); // loop flash
            }

        } else if (t >= 80) {
            robot.setGoBildaLight(0.33); // Orange
            flashActive = false;

        } else if (t >= 60) {
            // Yellow flash (60–80)
            if (!flashActive) {
                flash.resetTimer();
                flashActive = true;
            }

            double ft = flash.getElapsedTimeSeconds();

            if (ft < 0.333) {
                robot.setGoBildaLight(0.39);
            } else if (ft < 0.667) {
                robot.setGoBildaLight(0.0);
            } else if (ft < 1.0) {
                robot.setGoBildaLight(0.39);
            } else {
                flash.resetTimer();
            }

        } else {
            robot.setGoBildaLight(0.5); // Green
            flashActive = false;
        }

        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", actualRPM);
        telemetry.addData("LL Dis", robot.getDistance());
        telemetry.addData("Beam Broken?", robot.isBroken());
        telemetry.update();
    }

    /* ===================== UTILS ===================== */
    private double updateRPM() {
        return robot.calculateRegression();
    }
}