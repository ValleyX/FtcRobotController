/*
package org.firstinspires.ftc.team12841.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.team12841.RobotHardware;
import org.firstinspires.ftc.team12841.configs.PanelsConfig;
import org.firstinspires.ftc.team12841.pedroPathing.Constants;

@Autonomous(name = "Auto Template", group = "Autos")
public class AutoTemplate extends OpMode {

    */
/* ===================== CORE ===================== *//*


    private RobotHardware robot;
    private Follower follower;

    */
/* ===================== TIMERS ===================== *//*


    private Timer pathTimer;
    private Timer alignTimer;
    private Timer opmodeTimer;

    */
/* ===================== STATE ===================== *//*


    private int pathState = 0;

    */
/* ===================== FIELD POSES ===================== *//*


    private static final Pose START_POSE =
            new Pose(0, 0, Math.toRadians(180));

    private static final Pose SCORE_POSE =
            new Pose(0, 0, Math.toRadians(225));

    private static final Pose END_POSE =
            new Pose(0, 0, Math.toRadians(0));

    */
/* ===================== PATHS ===================== *//*


    private PathChain scorePreload;
    private PathChain endAuto;

    */
/* ===================== LIMELIGHT CONSTANTS ===================== *//*


    private static final double LL_MAX_ALIGN_TIME = 1.5; // seconds

    */
/* ===================== INIT ===================== *//*


    @Override
    public void init() {

        robot = new RobotHardware(this);

        pathTimer   = new Timer();
        alignTimer  = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);

        // TTPOS0

        buildPaths();

        telemetry.addLine("Auto Template Initialized");
        telemetry.update();
    }

    */
/* ===================== BUILD PATHS ===================== *//*


    private void buildPaths() {

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, SCORE_POSE))
                .setLinearHeadingInterpolation(
                        START_POSE.getHeading(),
                        SCORE_POSE.getHeading()
                )
                .build();

        endAuto = follower.pathBuilder()
                .addPath(new BezierLine(SCORE_POSE, END_POSE))
                .setLinearHeadingInterpolation(
                        SCORE_POSE.getHeading(),
                        END_POSE.getHeading()
                )
                .build();
    }

    */
/* ===================== START ===================== *//*


    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    */
/* ===================== LOOP ===================== *//*


    @Override
    public void loop() {

        follower.update();
        autonomousUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)",
                Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("LL tx", robot.getTx());
        telemetry.update();
    }

    */
/* ===================== FSM ===================== *//*


    private void autonomousUpdate() {

        switch (pathState) {

            */
/* -----------------------------
             * 0 – Drive to score preload
             * ----------------------------- *//*

            case 0:
                follower.followPath(scorePreload);
                // LINREG LOGIC
                setPathState(1);
                break;

            */
/* -----------------------------
             * 1 – Wait for path completion
             * ----------------------------- *//*

            case 1:
                if (!follower.isBusy()) {
                    alignTimer.resetTimer();
                    setPathState(2);
                }
                break;

            */
/* -----------------------------
             * 2 – Limelight align (using RH)
             * ----------------------------- *//*

            case 2:
                robot.updateLLHeading();

                if (robot.getTx() != -999) {
                    robot.turnToFree(
                            robot.headingDeg() + robot.getTx(),
                            1.0
                    );
                }

                if (Math.abs(robot.getTx()) < PanelsConfig.LL_ALIGN_TOLERANCE ||
                        alignTimer.getElapsedTimeSeconds() > LL_MAX_ALIGN_TIME) {

                    robot.addAlign(0, 0, 0, 0);
                    pathTimer.resetTimer();
                    setPathState(3);
                }
                break;

            */
/* -----------------------------
             * 3 – Shoot preload
             * ----------------------------- *//*

            case 3:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    fire();
                    pathTimer.resetTimer();
                    setPathState(4);
                }
                break;

            */
/* -----------------------------
             * 4 – Reset shooter, go park
             * ----------------------------- *//*

            case 4:
                if (pathTimer.getElapsedTimeSeconds() > 0.8) {
                    resetFlick();
                    follower.followPath(endAuto, true);
                    robot.stopShooter();
                    setPathState(5);
                }
                break;

            */
/* -----------------------------
             * 5 – End
             * ----------------------------- *//*

            case 5:
                // done
                break;
        }
    }

    */
/* ===================== HELPERS ===================== *//*


    private void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    */
/* ===================== MECHANISM PLACEHOLDERS ===================== *//*


    private void fire() {
    }

    private void resetFlick() {
    }
}
*/
