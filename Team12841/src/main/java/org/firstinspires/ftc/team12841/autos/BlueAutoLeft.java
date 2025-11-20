package org.firstinspires.ftc.team12841.autos;

import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.team12841.RobotHardware;
import org.firstinspires.ftc.team12841.teleOps.TeleOpPathing;
import org.firstinspires.ftc.team12841.configs.TeleOpConfig;
import org.firstinspires.ftc.team12841.pedroPathing.Constants;

@Autonomous(name = "Blue Left", group = "Autos")
public class BlueAutoLeft extends OpMode {

    private RobotHardware robot;
    private TeleOpPathing teleOpPathing;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    private final Pose startPose = new Pose(67, -5, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(48, -5, Math.toRadians(151)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose endPose = new Pose(48, -5, Math.toRadians(90)); // Highest (First Set) of Artifacts from the Spike Mark.
    private PathChain scorePreload, endAuto;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        endAuto = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                teleOpPathing.shootPwr = 0.725;
                robot.turntableServo.setPosition(TeleOpConfig.TT_POS_0);
                teleOpPathing.startShooter = true;
                sleep(2000);
                robot.shooterServo.setPosition(TeleOpConfig.SHOOTER_FIRE);
                sleep(500);
                robot.shooterServo.setPosition(TeleOpConfig.SHOOTER_IDLE);
                sleep(500);
                robot.turntableServo.setPosition(TeleOpConfig.TT_POS_1);
                sleep(750);
                robot.shooterServo.setPosition(TeleOpConfig.SHOOTER_FIRE);
                sleep(500);
                robot.shooterServo.setPosition(TeleOpConfig.SHOOTER_IDLE);
                sleep(500);
                robot.turntableServo.setPosition(TeleOpConfig.TT_POS_2);
                sleep(750);
                robot.shooterServo.setPosition(TeleOpConfig.SHOOTER_FIRE);
                sleep(500);
                robot.shooterServo.setPosition(TeleOpConfig.SHOOTER_IDLE);
                teleOpPathing.startShooter = false;
                setPathState(1);
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(endAuto, true);
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        teleOpPathing = new TeleOpPathing();
        robot = new RobotHardware(this);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}