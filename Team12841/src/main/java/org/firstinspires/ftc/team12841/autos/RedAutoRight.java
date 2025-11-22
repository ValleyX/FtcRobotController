package org.firstinspires.ftc.team12841.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.team12841.RobotHardware;
import org.firstinspires.ftc.team12841.configs.TeleOpConfig;
import org.firstinspires.ftc.team12841.pedroPathing.Constants;

@Autonomous(name = "Red Right", group = "Autos")
public class RedAutoRight extends OpMode {

    private RobotHardware robot;
    private Follower follower;
    private Timer pathTimer, alignTimer, opmodeTimer;

    private int pathState = 0;

    // Field poses
    private final Pose startPose = new Pose(63, 9, Math.toRadians(180));
    private final Pose scorePose = new Pose(46, 9, Math.toRadians(135));
    private final Pose endPose   = new Pose(36, 9, Math.toRadians(80));


    private PathChain scorePreload, endAuto;

    // Limelight / aiming constants
    private static final double LL_KP          = TeleOpConfig.KP; // your gain
    private static final double LL_MAX_TURN    = 0.16;            // max turn power
    private static final double LL_TOLERANCE   = 0.5;             // deg of tx considered “aligned”
    private static final double LL_MAX_ALIGN_S = 1.5;             // max time spent aligning

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
            // 0 – Start path + spin up shooter
            // --------------------------------
            case 0:
                follower.followPath(scorePreload);  // drive to score pose
                robot.shooterMotor.setPower(0.75);

                setPathState(2);
                break;

            // --------------------------------
            // 2 – Wait for path to finish
            // --------------------------------
            case 2:
                if (!follower.isBusy()) {
                    // Path finished; swap into teleop-drive mode for LL rotation
                    follower.startTeleopDrive();
                    alignTimer.resetTimer();
                    pathTimer.resetTimer();
                    setPathState(3);
                }
                break;

            // --------------------------------
            // 3 – Limelight aim using setTeleOpDrive
            // --------------------------------
            case 3: {
                double tx = robot.getTx();  // your LL horizontal offset

                // No target case: timeout and just shoot anyway
                if (tx == -999 && !follower.isBusy()) {
                    if (alignTimer.getElapsedTimeSeconds() > LL_MAX_ALIGN_S) {
                        // stop rotation
                        follower.setTeleOpDrive(0, 0, 0, true);
                        pathTimer.resetTimer();
                        setPathState(10);
                    }
                    break;
                }

                // P-control on tx
                double turn = -(tx * LL_KP); // keep the minus: turn toward target

                // clamp
                if (turn > LL_MAX_TURN)  turn = LL_MAX_TURN;
                if (turn < -LL_MAX_TURN) turn = -LL_MAX_TURN;

                // rotate in place, robot-centric
                follower.setTeleOpDrive(0, 0, turn, true);

                // aligned or timed out
                if (Math.abs(tx) <= LL_TOLERANCE ||
                        alignTimer.getElapsedTimeSeconds() > LL_MAX_ALIGN_S) {

                    follower.setTeleOpDrive(0, 0, 0, true); // stop rotating
                    pathTimer.resetTimer();
                    setPathState(10);
                }
                break;
            }

            // --------------------------------
            // 10–15 – Three-shot state machine
            // --------------------------------

            // 10 – initial wait before first shot
            case 10:
                if (pathTimer.getElapsedTimeSeconds() > 4) {
                    robot.shooterServo.setPosition(TeleOpConfig.SHOOTER_FIRE);
                    pathTimer.resetTimer();
                    setPathState(11);
                }
                break;

            // 11 – reset servo after first shot
            case 11:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    robot.shooterServo.setPosition(TeleOpConfig.SHOOTER_IDLE);
                    pathTimer.resetTimer();
                    setPathState(112);
                }
                break;

            // 112 – rotate turret to pos 1
            case 112:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    robot.turntableServo.setPosition(TeleOpConfig.TT_POS_1);
                    pathTimer.resetTimer();
                    setPathState(12);
                }
                break;

            // 12 – second shot
            case 12:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    robot.shooterServo.setPosition(TeleOpConfig.SHOOTER_FIRE);
                    pathTimer.resetTimer();
                    setPathState(13);
                }
                break;

            // 13 – reset after second shot
            case 13:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    robot.shooterServo.setPosition(TeleOpConfig.SHOOTER_IDLE);
                    pathTimer.resetTimer();
                    setPathState(132);
                }
                break;

            // 132 – rotate turret to pos 2
            case 132:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    robot.turntableServo.setPosition(TeleOpConfig.TT_POS_2);
                    pathTimer.resetTimer();
                    setPathState(14);
                }
                break;

            // 14 – third shot
            case 14:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    robot.shooterServo.setPosition(TeleOpConfig.SHOOTER_FIRE);
                    pathTimer.resetTimer();
                    setPathState(15);
                }
                break;

            // 15 – reset servo, then move on
            case 15:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    robot.shooterServo.setPosition(TeleOpConfig.SHOOTER_IDLE);
                    pathTimer.resetTimer();
                    setPathState(1);
                }
                break;

            // --------------------------------
            // 1 – After preload: drive end path
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
        // Pedro still needs this every cycle
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("tx", robot.getTx());
        telemetry.update();
    }

    // ---------------------------
    // INIT
    // ---------------------------
    @Override
    public void init() {
        robot = new RobotHardware(this);
        pathTimer = new Timer();
        alignTimer = new Timer();
        opmodeTimer = new Timer();
        robot.innit(2);

        follower = Constants.createFollower(hardwareMap);
        robot.turntableServo.setPosition(TeleOpConfig.TT_POS_0);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
}
