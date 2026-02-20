package org.firstinspires.ftc.team12841.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.team12841.RobotHardware;

@Autonomous(name="Red Far", group="AUTO")
public class RedAutoFar extends OpMode {

    private Follower follower;
    private RobotHardware robot;
    private Timer pathTimer;
    private Timer pauseTimer;

    private int state = 0;

    private static final double NORMAL_DRIVE_POWER = 0.7;
    private static final double INTAKE_DRIVE_POWER = 0.8;
    private static final double WIGGLE_PAUSE = 1;

    // ---------------- PATHS ----------------
    private PathChain START_TO_SHOOT;

    private PathChain SHOOT_TO_ALIGN1;
    private PathChain ALIGN1_TO_INTAKE1;
    private PathChain INTAKE1_TO_SHOOT;

    private PathChain SHOOT_TO_ALIGN2;
    private PathChain ALIGN2_TO_INTAKE2;
    private PathChain INTAKE2_TO_SHOOT;

    private PathChain SHOOT_TO_PARK;

    private boolean runIntake = false;
    private boolean intakeCaptured = false;
    private boolean aligning = false;

    // ---------------- CONTROL ----------------

    private void shootFlick(double power) {
        if (!runIntake) {
            robot.intake.setPower(1);
            robot.flick.setPower(power);
        }
    }

    private void intakeFlick(double power) {
        robot.flick.setPower(power);
    }

    private void align() {
        robot.alignWithLimelight(-1);
    }

    private void runIntakeForward() {
        boolean beamBroken = !robot.isBroken();
        robot.intake.setPower(1);

        if (!intakeCaptured) {
            if (!beamBroken) {
                intakeFlick(-1);
            } else {
                intakeCaptured = true;
                intakeFlick(0);
            }
        }
    }

    private void stopIntake() {
        runIntake = false;
        robot.intake.setPower(0);
        intakeFlick(0);
    }

    private void wiggleLeft() {
        follower.turn(Math.toRadians(10), true);
    }

    @Override
    public void init() {
        robot = new RobotHardware(this);
        follower = robot.getFollower();
        pathTimer = new Timer();
        pauseTimer = new Timer();

        robot.limelight.pipelineSwitch(2);

        Pose startPose = new Pose(0, 0, 0);
        Pose shootPreloadPose = new Pose(9.6771, 3.4543, -0.3651);

        Pose alignIntake1 = new Pose(31.0451, -5.3002, -1.6138);
        Pose intake1Pose = new Pose(35.7305, -44.3417, -1.6798);
        Pose shoot1Pose = new Pose(9, -2.5734, -0.3018);

        Pose alignIntake2 = new Pose(35, -45, -Math.PI);
        Pose intake2Pose = new Pose(9, -45, -Math.PI);
        Pose shoot2Pose = new Pose(15, -12, -0.3651);

        Pose parkPose = new Pose(30.2975, -15, -1.6033);

        follower.setStartingPose(startPose);

        START_TO_SHOOT = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPreloadPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPreloadPose.getHeading())
                .build();

        SHOOT_TO_ALIGN1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPreloadPose, alignIntake1))
                .setLinearHeadingInterpolation(shootPreloadPose.getHeading(), alignIntake1.getHeading())
                .build();

        ALIGN1_TO_INTAKE1 = follower.pathBuilder()
                .addPath(new BezierLine(alignIntake1, intake1Pose))
                .setLinearHeadingInterpolation(alignIntake1.getHeading(), intake1Pose.getHeading())
                .build();

        INTAKE1_TO_SHOOT = follower.pathBuilder()
                .addPath(new BezierLine(intake1Pose, shoot1Pose))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), shoot1Pose.getHeading())
                .build();

        SHOOT_TO_ALIGN2 = follower.pathBuilder()
                .addPath(new BezierLine(shoot1Pose, alignIntake2))
                .setLinearHeadingInterpolation(shoot1Pose.getHeading(), alignIntake2.getHeading())
                .build();

        ALIGN2_TO_INTAKE2 = follower.pathBuilder()
                .addPath(new BezierLine(alignIntake2, intake2Pose))
                .setLinearHeadingInterpolation(alignIntake2.getHeading(), intake2Pose.getHeading())
                .build();

        INTAKE2_TO_SHOOT = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, shoot2Pose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), shoot2Pose.getHeading())
                .build();

        SHOOT_TO_PARK = follower.pathBuilder()
                .addPath(new BezierLine(shoot2Pose, parkPose))
                .setLinearHeadingInterpolation(shoot2Pose.getHeading(), parkPose.getHeading())
                .build();
    }

    @Override
    public void start() {
        state = 0;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();

        switch (state) {

            case 0:
                follower.setMaxPower(NORMAL_DRIVE_POWER);
                robot.setShooterRPM(robot.calculateRegression());
                follower.followPath(START_TO_SHOOT);
                state++;
                break;

            case 1:
                if (!follower.isBusy()) {
                    aligning = true;
                    pathTimer.resetTimer();
                    state++;
                }
                break;

            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 2.5)
                {
                    stopIntake();
                    shootFlick(-1);
                }
                if (pathTimer.getElapsedTimeSeconds() > 4) {
                    shootFlick(0);
                    aligning = false;
                    follower.followPath(SHOOT_TO_ALIGN1);
                    intakeCaptured = false;
                    state++;
                }
                break;

            case 3:
                runIntake = true;
                if (!follower.isBusy()) {
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    follower.followPath(ALIGN1_TO_INTAKE1);
                    pathTimer.resetTimer();
                    state++;
                }
                break;

            case 4:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.5) {
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
                    aligning = true;
                    pathTimer.resetTimer();
                    state++;
                }
                break;

            case 7:
                if (pathTimer.getElapsedTimeSeconds() > 2)
                {
                    stopIntake();
                    shootFlick(-1);
                }
                if (pathTimer.getElapsedTimeSeconds() > 4) {
                    shootFlick(0);
                    aligning = false;
                    follower.followPath(SHOOT_TO_ALIGN2);
                    intakeCaptured = false;
                    state++;
                }
                break;

            case 8:
                runIntake = true;
                if (!follower.isBusy()) {
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    follower.followPath(ALIGN2_TO_INTAKE2);
                    pathTimer.resetTimer();
                    state++;
                }
                break;

            case 9:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 1.5) {
                    pauseTimer.resetTimer();
                    state++;
                }
                break;

            case 10:
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    follower.followPath(INTAKE2_TO_SHOOT);
                    state++;
                break;

            case 11:
                if (!follower.isBusy()) {
                    aligning = true;
                    pathTimer.resetTimer();
                    state++;
                }
                break;

            case 12:
                if (pathTimer.getElapsedTimeSeconds() > 2)
                {
                    stopIntake();
                    shootFlick(-1);
                }
                if (pathTimer.getElapsedTimeSeconds() > 4) {
                    shootFlick(0);
                    aligning = false;
                    follower.followPath(SHOOT_TO_PARK);
                    state++;
                }
                break;

            default:
                stopIntake();
                robot.shooter.setVelocity(0);
                break;
        }

        if (aligning) align();
        if (runIntake) runIntakeForward();

        telemetry.addData("State", state);
        telemetry.update();
    }
}
