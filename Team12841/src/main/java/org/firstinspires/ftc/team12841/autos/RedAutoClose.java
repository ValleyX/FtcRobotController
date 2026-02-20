package org.firstinspires.ftc.team12841.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.team12841.RobotHardware;

@Autonomous(name = "Red Close", group = "AUTO")
public class RedAutoClose extends OpMode {

    private Follower follower;
    private RobotHardware robot;
    private Timer pathTimer;
    private Timer pauseTimer;

    private int state = 0;

    private static final double NORMAL_DRIVE_POWER = 0.6;
    private static final double INTAKE_DRIVE_POWER = 0.8;
    private static final double WIGGLE_PAUSE = 1;

    private PathChain START_TO_SHOOT;

    private PathChain SHOOT_TO_ALIGN1;
    private PathChain ALIGN1_TO_INTAKE1;
    private PathChain INTAKE1_TO_SHOOT;

    private PathChain SHOOT_TO_ALIGN2;
    private PathChain ALIGN2_TO_INTAKE2;
    private PathChain INTAKE2_TO_SHOOT;

    private PathChain SHOOT_AFTER_INTAKE2;
    private PathChain SHOOT_FAR_TO_PARK;

    private boolean runIntake = false;
    private boolean intakeCaptured = false;
    private boolean aligning = false;

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
        boolean beamBroken = !robot.isBroken(); // active low
        robot.intake.setPower(1);

        if (!intakeCaptured) {
            if (!beamBroken) {
                intakeFlick(-1);
            } else {
                intakeCaptured = true;
                intakeFlick(0);
            }
        } else {
            intakeFlick(0);
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

        Pose startPose = new Pose(0, 0, 0);
        Pose shootPreloadPose = new Pose(-48.24, -6, -0.0621);

        Pose alignIntake1 = new Pose(-41.878, -11.635, -0.7688);
        Pose intake1Pose = new Pose(-20.81, -27.279, -0.7956);
        Pose shoot1Pose = new Pose(-45.53, -11, 0.0724);

        Pose alignIntake2 = new Pose(-53.6084, -20.4814, -0.823);
        Pose intake2Pose = new Pose(-32.1654, -42.2741, -0.823);
        Pose shoot2Pose = new Pose(-47.7855, -5.4689, -0.088);

        Pose parkPose = new Pose(-28.5896, 11.6066, -0.846);

        follower.setStartingPose(startPose);

        START_TO_SHOOT = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPreloadPose))
                .setLinearHeadingInterpolation(
                        startPose.getHeading(),
                        shootPreloadPose.getHeading()
                )
                .build();

        SHOOT_TO_ALIGN1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPreloadPose, alignIntake1))
                .setLinearHeadingInterpolation(
                        shootPreloadPose.getHeading(),
                        alignIntake1.getHeading()
                )
                .build();

        ALIGN1_TO_INTAKE1 = follower.pathBuilder()
                .addPath(new BezierLine(alignIntake1, intake1Pose))
                .setLinearHeadingInterpolation(
                        alignIntake1.getHeading(),
                        intake1Pose.getHeading()
                )
                .build();

        INTAKE1_TO_SHOOT = follower.pathBuilder()
                .addPath(new BezierLine(intake1Pose, shoot1Pose))
                .setLinearHeadingInterpolation(
                        intake1Pose.getHeading(),
                        shoot1Pose.getHeading()
                )
                .build();

        SHOOT_TO_ALIGN2 = follower.pathBuilder()
                .addPath(new BezierLine(shoot1Pose, alignIntake2))
                .setLinearHeadingInterpolation(
                        shoot1Pose.getHeading(),
                        alignIntake2.getHeading()
                )
                .build();

        ALIGN2_TO_INTAKE2 = follower.pathBuilder()
                .addPath(new BezierLine(alignIntake2, intake2Pose))
                .setLinearHeadingInterpolation(
                        alignIntake2.getHeading(),
                        intake2Pose.getHeading()
                )
                .build();

        INTAKE2_TO_SHOOT = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, shoot2Pose))
                .setLinearHeadingInterpolation(
                        intake2Pose.getHeading(),
                        shoot2Pose.getHeading()
                )
                .build();

        SHOOT_AFTER_INTAKE2 = follower.pathBuilder()
                .addPath(new BezierLine(shoot2Pose, shoot1Pose))
                .setLinearHeadingInterpolation(
                        shoot2Pose.getHeading(),
                        shoot1Pose.getHeading()
                )
                .build();

        SHOOT_FAR_TO_PARK = follower.pathBuilder()
                .addPath(new BezierLine(shoot2Pose, parkPose))
                .setLinearHeadingInterpolation(
                        shoot2Pose.getHeading(),
                        parkPose.getHeading()
                )
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
                if (pathTimer.getElapsedTimeSeconds() > 1.3)
                {
                    stopIntake();
                    shootFlick(-1);
                }
                if (pathTimer.getElapsedTimeSeconds() > 3.3) {
                    aligning = false;
                    shootFlick(0);
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
                if (pathTimer.getElapsedTimeSeconds() > 1.3)
                {
                    stopIntake();
                    shootFlick(-1);
                }
                if (pathTimer.getElapsedTimeSeconds() > 3.3) {
                    shootFlick(0);
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
                    robot.alignWithLimelight(-1);
                    pathTimer.resetTimer();
                    state++;
                }
                break;

            case 13:
                if (pathTimer.getElapsedTimeSeconds() > 1.3)
                {
                    stopIntake();
                    shootFlick(-1);
                }
                if (pathTimer.getElapsedTimeSeconds() > 3.3) {
                    shootFlick(0);
                    follower.followPath(SHOOT_FAR_TO_PARK);
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
