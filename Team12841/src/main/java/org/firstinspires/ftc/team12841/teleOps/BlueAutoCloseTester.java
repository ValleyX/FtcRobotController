package org.firstinspires.ftc.team12841.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team12841.RobotHardware;

@TeleOp(name="Blue Close (FULL FIXED)", group="AUTO")
public class BlueAutoCloseTester extends OpMode {

    private Follower follower;
    private RobotHardware robot;
    private Timer pathTimer;

    private int state = 0;

    private PathChain START_TO_SHOOT;
    private PathChain SHOOT_TO_ALIGN1;
    private PathChain ALIGN1_TO_INTAKE1;
    private PathChain INTAKE1_TO_SHOOT;

    private PathChain SHOOT_TO_ALIGN2;
    private PathChain ALIGN2_TO_INTAKE2;
    private PathChain INTAKE2_TO_SHOOT;

    private PathChain SHOOT_TO_ALIGN3;
    private PathChain ALIGN3_TO_INTAKE3;

    private PathChain INTAKE3_TO_SHOOT_FAR;
    private PathChain SHOOT_FAR_TO_PARK;

    // ---------------- INTAKE LATCH ----------------
    private boolean runIntake = false;
    private boolean intakeCaptured = false;
    private boolean lastBeamBroken = false;

    // ---------------- FLICK CONTROL ----------------

    private void shootFlick(double power) {
        if (!runIntake) {
            robot.flick.setPower(power);
        }
    }

    private void intakeFlick(double power) {
        robot.flick.setPower(power);
    }

    // ---------------- INTAKE ----------------

    private void runIntakeForward() {
        boolean beamBroken = robot.isBroken(); // true = note present
        intakeFlick(beamBroken ? 0 : -1);
        robot.intake.setPower(1);
    }

    private void stopIntake() {
        runIntake = false;
        robot.intake.setPower(0);
        intakeFlick(0);
    }

    // ---------------- INIT ----------------

    @Override
    public void init() {
        robot = new RobotHardware(this);
        follower = robot.getFollower();
        pathTimer = new Timer();

        Pose startPose = new Pose(0, 0, 0);
        Pose shootPreloadPose = new Pose(-43.7, 0.5, 0.1);

        Pose alignIntake1 = new Pose(-46.24, 4.47, 0.79);
        Pose intake1Pose = new Pose(-19.46, 30.81, 0.78);
        Pose shoot1Pose = new Pose(-55.9, 7.37, 0.021);

        Pose alignIntake2 = new Pose(-56.4, 21, 0.85);
        Pose intake2Pose = new Pose(-34.43, 45.38, 0.85);
        Pose shoot2Pose = new Pose(-54.35, 14.66, 0.002);

        Pose alignIntake3 = new Pose(-70.96, 34.38, 0.80);
        Pose intake3Pose = new Pose(-38.39, 58.36, 0.85);

        Pose shootFarPose = new Pose(-86.82, 41.8, -0.355);
        Pose parkPose = new Pose(-66.48, 32.19, -0.703);

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

        SHOOT_TO_ALIGN3 = follower.pathBuilder()
                .addPath(new BezierLine(shoot2Pose, alignIntake3))
                .setLinearHeadingInterpolation(shoot2Pose.getHeading(), alignIntake3.getHeading())
                .build();

        ALIGN3_TO_INTAKE3 = follower.pathBuilder()
                .addPath(new BezierLine(alignIntake3, intake3Pose))
                .setLinearHeadingInterpolation(alignIntake3.getHeading(), intake3Pose.getHeading())
                .build();

        INTAKE3_TO_SHOOT_FAR = follower.pathBuilder()
                .addPath(new BezierLine(intake3Pose, shootFarPose))
                .setLinearHeadingInterpolation(intake3Pose.getHeading(), shootFarPose.getHeading())
                .build();

        SHOOT_FAR_TO_PARK = follower.pathBuilder()
                .addPath(new BezierLine(shootFarPose, parkPose))
                .setLinearHeadingInterpolation(shootFarPose.getHeading(), parkPose.getHeading())
                .build();
    }

    // ---------------- START ----------------

    @Override
    public void start() {
        state = 0;
        pathTimer.resetTimer();
    }

    // ---------------- LOOP ----------------

    @Override
    public void loop() {
        follower.update();

        switch (state) {

            case 0:
                robot.shooter.setVelocity(2500);
                follower.followPath(START_TO_SHOOT);
                pathTimer.resetTimer();
                if(gamepad1.aWasPressed())
                    ;
            {
                state++;break;
            }

            case 1:
                if (!follower.isBusy()) {
                    robot.alignWithLimelight(-1);
                    pathTimer.resetTimer();
                    gamepad1.aWasPressed();
                    {
                        state++;
                    }
                }
                break;

            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    intakeCaptured = false; // NOTE IS LEAVING
                    shootFlick(-1);
                }
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    shootFlick(0);
                    follower.followPath(SHOOT_TO_ALIGN1);
                    if(gamepad1.aWasPressed());
                    {
                        state++;
                    }
                }
                break;

            case 3:
                runIntake = true;//JAE NOTE TO TANNER - try calling intake directly, don't set this bool.  The
                //program has to wait before it check Beam Break until the case is done and then gets to bottom of switch statement
                if (!follower.isBusy()) {
                    follower.followPath(ALIGN1_TO_INTAKE1);
                    if(gamepad1.aWasPressed());
                    {
                        state++;
                    }
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    stopIntake();
                    follower.followPath(INTAKE1_TO_SHOOT);
                    if(gamepad1.aWasPressed());
                    {
                        state++;
                    }
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    robot.shooter.setVelocity(2500);
                    robot.alignWithLimelight(-1);
                    pathTimer.resetTimer();
                    if(gamepad1.aWasPressed());
                    {
                        state++;
                    }
                }
                break;

            case 6:
                if (pathTimer.getElapsedTimeSeconds() > 0.8) {
                    intakeCaptured = false;
                    shootFlick(-1);
                }
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    shootFlick(0);
                    follower.followPath(SHOOT_TO_ALIGN2);
                    if(gamepad1.aWasPressed());
                    {
                        state++;
                    }
                }
                break;

            case 7:
                runIntake = true;
                if (!follower.isBusy()) {
                    follower.followPath(ALIGN2_TO_INTAKE2);
                    if(gamepad1.aWasPressed());
                    {
                        state++;
                    }
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    stopIntake();
                    follower.followPath(INTAKE2_TO_SHOOT);
                    if(gamepad1.aWasPressed());
                    {
                        state++;
                    }
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    robot.shooter.setVelocity(2500);
                    robot.alignWithLimelight(-1);
                    pathTimer.resetTimer();
                    if(gamepad1.aWasPressed());
                    {
                        state++;
                    }
                }
                break;

            case 10:
                if (pathTimer.getElapsedTimeSeconds() > 0.8) {
                    intakeCaptured = false;
                    shootFlick(-1);
                }
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    shootFlick(0);
                    follower.followPath(SHOOT_TO_ALIGN3);
                    if(gamepad1.aWasPressed());
                    {
                        state++;
                    }
                }
                break;

            case 11:
                runIntake = true;
                if (!follower.isBusy()) {
                    follower.followPath(ALIGN3_TO_INTAKE3);
                    if(gamepad1.aWasPressed());
                    {
                        state++;
                    }
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    stopIntake();
                    follower.followPath(INTAKE3_TO_SHOOT_FAR);
                    if(gamepad1.aWasPressed());
                    {
                        state++;
                    }
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    robot.shooter.setVelocity(3900);
                    robot.alignWithLimelight(-1);
                    pathTimer.resetTimer();
                    if(gamepad1.aWasPressed());
                    {
                        state++;
                    }
                }
                break;

            case 14:
                if (pathTimer.getElapsedTimeSeconds() > 0.8) {
                    intakeCaptured = false;
                    shootFlick(-1);
                }
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    shootFlick(0);
                    follower.followPath(SHOOT_FAR_TO_PARK);
                    if(gamepad1.aWasPressed());
                    {
                        state++;
                    }
                }
                break;

            default:
                stopIntake();
                robot.shooter.setVelocity(0);
                break;
        }

        if (runIntake) {
            runIntakeForward();
        }

        telemetry.addData("State", state);
        telemetry.addData("Beam", robot.isBroken());
        telemetry.addData("Captured", intakeCaptured);
        telemetry.addData("Run Intake", runIntake);
        telemetry.update();
    }
}
