package org.firstinspires.ftc.team12841.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.RobotHardware;

@Disabled
@Autonomous(name = "Auto Base", group = "AUTO")
public class AutoBase extends LinearOpMode {

    int pipeline = 0;
    Follower follower;
    private RobotHardware robot;
    private Timer pathTimer;
    private Timer pauseTimer;

    private int state = 0;

    private static final double NORMAL_DRIVE_POWER = 0.6;
    private static final double INTAKE_DRIVE_POWER = 0.8;
    private static final double WIGGLE_PAUSE = 1;

    PathChain START_TO_SHOOT;

    PathChain SHOOT_TO_ALIGN1;
    PathChain ALIGN1_TO_INTAKE1;
    PathChain INTAKE1_TO_SHOOT;

    PathChain SHOOT_TO_ALIGN2;
    PathChain ALIGN2_TO_INTAKE2;
    PathChain INTAKE2_TO_SHOOT;

    PathChain SHOOT_AFTER_INTAKE2;
    PathChain SHOOT_FAR_TO_PARK;

    private boolean runIntake = false;
    private boolean intakeCaptured = false;
    private boolean aligning = false;

    // --- HELPER METHODS ADAPTED TO NEW HARDWARE --- //

    private void startShooting() {
        if (!runIntake) {
            robot.stopBallRelease(); // Open blocker
            robot.feed();            // Feed the ball
        }
    }

    private void stopShooting() {
        robot.stopBallHold();
        robot.stopFeed();
    }

    private void align() {
        robot.alignWithLimelight(-1);

        // Continously update hood and RPM while aligning based on regression
        if (robot.getTx() != -999) {
            robot.setShooterRPM(robot.calculateRegression());
        }
        robot.aimHood(robot.getHoodAim(robot.getBotDis()));
    }

    private void runIntakeForward() {
        boolean beamBroken = robot.isBroken(); // uses isBroken() from RH

        if (!intakeCaptured) {
            if (!beamBroken) {
                robot.intake(1.0);
            } else {
                intakeCaptured = true;
                robot.intake(0);
            }
        } else {
            robot.intake(0);
        }
    }

    private void stopIntake() {
        runIntake = false;
        robot.intake(0);
    }

    private void wiggleLeft() {
        // Assuming your Pedro version supports this directly as in the template
        follower.turn(Math.toRadians(10), true);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Hardware (Using the secondary constructor for auto)
        robot = new RobotHardware(this, true);
        follower = robot.getFollower();

        pathTimer = new Timer();
        pauseTimer = new Timer();

        // Init Limelight pipeline
        robot.innit(pipeline);

        /* * TODO: Build PathChains Here
         */

        telemetry.addLine("Auto Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        state = 0;
        pathTimer.resetTimer();

        // MAIN AUTO LOOP
        while (opModeIsActive()) {
            follower.update();

            switch (state) {
                case 0:
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    robot.setShooterRPM(2800);
                    follower.followPath(START_TO_SHOOT);
                    state++;
                    break;

                case 1:
                    if (!follower.isBusy()) {
                        aligning = true;
                        pathTimer.resetTimer();
                        robot.setShooterRPM(robot.calculateRegression());
                        state++;
                    }
                    break;

                case 2:
                    if (pathTimer.getElapsedTimeSeconds() > 4.3) {
                        stopIntake();
                        startShooting();
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 6.3) {
                        aligning = false;
                        stopShooting();
                        intakeCaptured = false;
                        follower.followPath(SHOOT_TO_ALIGN1);
                        state++;
                    }
                    break;

                case 3:
                    runIntake = true;
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    if (!follower.isBusy()) {
                        follower.followPath(ALIGN1_TO_INTAKE1);
                        state++;
                    }
                    break;

                case 4:
                    if (!follower.isBusy()) {
                        wiggleLeft();
                        pauseTimer.resetTimer();
                        state++;
                    }
                    break;

                case 5:
                    if (pauseTimer.getElapsedTimeSeconds() > WIGGLE_PAUSE) {
                        follower.setMaxPower(NORMAL_DRIVE_POWER);
                        follower.followPath(INTAKE1_TO_SHOOT);
                        state++;
                    }
                    break;

                case 6:
                    if (!follower.isBusy()) {
                        robot.setShooterRPM(robot.calculateRegression());
                        aligning = true;
                        pathTimer.resetTimer();
                        state++;
                    }
                    break;

                case 7:
                    if (pathTimer.getElapsedTimeSeconds() > 1.3) {
                        stopIntake();
                        startShooting();
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 3.3) {
                        stopShooting();
                        aligning = false;
                        intakeCaptured = false;
                        follower.followPath(SHOOT_TO_ALIGN2);
                        state++;
                    }
                    break;

                case 8:
                    runIntake = true;
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    if (!follower.isBusy()) {
                        follower.followPath(ALIGN2_TO_INTAKE2);
                        state++;
                    }
                    break;

                case 9:
                    if (!follower.isBusy()) {
                        wiggleLeft();
                        pauseTimer.resetTimer();
                        state++;
                    }
                    break;

                case 10:
                    if (pauseTimer.getElapsedTimeSeconds() > WIGGLE_PAUSE) {
                        follower.setMaxPower(NORMAL_DRIVE_POWER);
                        follower.followPath(INTAKE2_TO_SHOOT);
                        state++;
                    }
                    break;

                case 11:
                    if (!follower.isBusy()) {
                        robot.setShooterRPM(robot.calculateRegression());
                        follower.followPath(SHOOT_AFTER_INTAKE2);
                        state++;
                    }
                    break;

                case 12:
                    if (!follower.isBusy()) {
                        robot.alignWithLimelight(-1); // Start align
                        pathTimer.resetTimer();
                        state++;
                    }
                    break;

                case 13:
                    if (pathTimer.getElapsedTimeSeconds() > 1.3) {
                        stopIntake();
                        startShooting();
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 3.3) {
                        stopShooting();
                        follower.followPath(SHOOT_FAR_TO_PARK);
                        state++;
                    }
                    break;

                default:
                    stopIntake();
                    robot.stopShooter();
                    break;
            }

            if (aligning) align();
            if (runIntake) runIntakeForward();

            telemetry.addData("State", state);
            telemetry.addData("Intake Captured", intakeCaptured);
            telemetry.addData("Limelight Distance", robot.getBotDis());
            telemetry.update();
        }
    }
}