package org.firstinspires.ftc.team12841.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.team12841.RobotHardware;
import org.firstinspires.ftc.team12841.teleOps.TeleOpPathing;
import org.firstinspires.ftc.team12841.configs.TeleOpConfig;
import org.firstinspires.ftc.team12841.pedroPathing.Constants;

@Autonomous(name = "Blue Left", group = "Autos")
public class BlueAutoLeft extends OpMode {

    private RobotHardware robot;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private int pathState = 0;

    private final Pose startPose = new Pose(48, -5, Math.toRadians(90));
    private final Pose scorePose = new Pose(67, -5, Math.toRadians(61));
    private final Pose endPose   = new Pose(67, -5, Math.toRadians(90));

    private PathChain scorePreload, endAuto;

    // ---------------------------
    // BUILD PATHS
    // ---------------------------
    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        endAuto = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
                .build();
    }


    // ---------------------------
    // AUTONOMOUS STATE MACHINE
    // ---------------------------
    public void autonomousPathUpdate() {
        switch (pathState) {

            // --------------------------------
            // STATE 0 – DRIVE TO SCORE & SPINUP SHOOTER
            // --------------------------------
            case 0:
                follower.followPath(scorePreload);
                //double rotCmd = 0;
                //double tx = robot.getTx(); // NEGATIVE stays negative
                //if (tx != -999) {

                    //double Kp = TeleOpConfig.KP;
                    //double turn = -(tx * Kp);  // ← KEEP NEGATIVE

                    //double maxTurn = 0.16;
                    //if (turn > maxTurn) turn = maxTurn;
                    //if (turn < -maxTurn) turn = -maxTurn;

                    //rotCmd += turn;
                //}
                robot.shooterMotor.setPower(0.725);
                robot.turntableServo.setPosition(TeleOpConfig.TT_POS_0);

                pathTimer.resetTimer();
                setPathState(10);
                break;


            // --------------------------------
            // STATE 10 – FIRST SHOT DELAY
            // --------------------------------
            case 10:
                if (pathTimer.getElapsedTimeSeconds() > 4) {
                    robot.shooterServo.setPosition(TeleOpConfig.SHOOTER_FIRE);
                    pathTimer.resetTimer();
                    setPathState(11);
                }
                break;

            case 11:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    robot.shooterServo.setPosition(TeleOpConfig.SHOOTER_IDLE);
                    pathTimer.resetTimer();
                    setPathState(112);
                }
                break;

            case 112:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    robot.turntableServo.setPosition(TeleOpConfig.TT_POS_1);
                    pathTimer.resetTimer();
                    setPathState(12);
                }
                break;
            case 12:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    robot.shooterServo.setPosition(TeleOpConfig.SHOOTER_FIRE);
                    pathTimer.resetTimer();
                    setPathState(13);
                }
                break;

            case 13:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    robot.shooterServo.setPosition(TeleOpConfig.SHOOTER_IDLE);
                    pathTimer.resetTimer();
                    setPathState(132);
                }
                break;

            case 132:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    robot.turntableServo.setPosition(TeleOpConfig.TT_POS_2);
                    pathTimer.resetTimer();
                    setPathState(14);
                }
                break;
            case 14:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    robot.shooterServo.setPosition(TeleOpConfig.SHOOTER_FIRE);
                    pathTimer.resetTimer();
                    setPathState(15);
                }
                break;

            case 15:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    robot.shooterServo.setPosition(TeleOpConfig.SHOOTER_IDLE);
                    pathTimer.resetTimer();
                    setPathState(1);
                }
                break;

            // --------------------------------
            // STATE 1 – AFTER PRELOAD
            // --------------------------------
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(endAuto, true);
                    robot.shooterMotor.setPower(0);
                }
                break;
        }
    }


    // ---------------------------
    // STATE HELPER
    // ---------------------------
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }


    // ---------------------------
    // LOOP
    // ---------------------------
    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }


    // ---------------------------
    // INIT
    // ---------------------------
    @Override
    public void init() {
        robot = new RobotHardware(this);
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();
    }


    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
}
