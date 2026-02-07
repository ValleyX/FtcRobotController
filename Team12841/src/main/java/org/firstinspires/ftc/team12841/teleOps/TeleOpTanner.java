package org.firstinspires.ftc.team12841.teleOps;

import com.bylazar.panels.Panels;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team12841.RobotHardware;
import org.firstinspires.ftc.team12841.configs.PanelsConfig;

@TeleOp(name = "TeleOpMain", group = "MAIN")
public class TeleOpTanner extends OpMode {

    /* ===================== HARDWARE ===================== */

    private RobotHardware robot;
    private Follower follower;

    /* ===================== PEDRO ===================== */

    private boolean poseReady = false;

    /* ===================== CONSTANTS ===================== */

    private static final double MAX_RPM  = 6000.0;
    private static final double RPM_STEP = 50.0;

    // Limelight rotation tuning
    private static final double LL_KP      = 0.04;
    private static final double LL_MAX_ROT = 0.6;


    /* ===================== STATE ===================== */

    private double  targetRPM      = 2800.0;
    private boolean shooterEnabled = false;

    // Button edge tracking
    private boolean dpadUpLast   = false;
    private boolean dpadDownLast = false;
    private boolean aLast        = false;

    public boolean babyMode = false;


    /* ===================== INIT ===================== */

    @Override
    public void init() {
        robot    = new RobotHardware(this);
        follower = robot.getFollower();

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

        if(gamepad1.startWasPressed())
        {
            babyMode = !babyMode;
        }

        /* ---------- LIMELIGHT ALIGN ---------- */

        if (gamepad1.left_trigger > 0.2) {
            robot.alignWithLimelight(-1);
        } else if (babyMode) {
            follower.setTeleOpDrive((forward * PanelsConfig.BABY), (strafe * PanelsConfig.BABY), (rotate * PanelsConfig.BABY), false);
        } else if (!babyMode) {
            follower.setTeleOpDrive(forward, strafe, rotate, false);
        }

        /* ---------- INTAKE ---------- */

        if (gamepad1.right_trigger > 0.2 && !robot.isBroken()){
            robot.intake.setPower(1);
            robot.flick.setPower(-1);
        } else if (gamepad1.right_trigger > 0.2 && robot.isBroken()) {
            robot.intake.setPower(1);
        } else if (gamepad1.b) {
                robot.intake.setPower(-1);
        } else if (gamepad1.x){
            robot.flick.setPower(-1);
        } else if (gamepad1.y){
            robot.flick.setPower(1);
        } else {
            robot.intake.setPower(0);
            robot.flick.setPower(0);
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

        dpadUpLast   = gamepad1.dpad_up;
        dpadDownLast = gamepad1.dpad_down;

        //if (gamepad1.dpadUpWasPressed())
        //    targetRPM = targetRPM + 100;

        //if (gamepad1.dpadDownWasPressed())
        //{
        //    targetRPM = targetRPM - 100;
        //}


        /* ---------- SHOOTER STUFF ---------- */

        if(gamepad1.x){
            robot.flick.setPower(-1);
        } else if(gamepad1.y){
            robot.flick.setPower(1);
        }else {
            robot.flick.setPower(0);
        }

        /* ---------- TELEMETRY ---------- */

        double actualRPM =
                (robot.shooter.getVelocity() * 60.0) /
                        RobotHardware.SHOOTER_TICKS_PER_REV;

        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", actualRPM);
        telemetry.addData("LL Dis", robot.getDistance());
        telemetry.addData("Beam Broken?", robot.isBroken());
        telemetry.update();
    }


    /* ===================== UTILS ===================== */

    private double updateRPM() {
        return robot.calculateRegression(robot.getDistance());
    }
}
