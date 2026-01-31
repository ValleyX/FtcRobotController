package org.firstinspires.ftc.team12841.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.team12841.RobotHardware;

@Autonomous(name="Blue Close (NEEDS TESTING)", group="AUTO")
public class BlueAutoClose extends OpMode {

    private Follower follower;
    private RobotHardware robot;

    private Timer pathTimer;

    private int state = 0;

    // Path definitions
    private PathChain START_TO_SHOOT;
    private PathChain INTAKE1;
    private PathChain INTAKE1_TO_SHOOT;
    private PathChain SHOOT1_TO_INTAKE2_ALIGN;
    private PathChain INTAKE2;
    private PathChain INTAKE2_TO_SHOOT;
    private PathChain MOVE_TO_CLEAR;
    private PathChain CLEAR_TO_INTAKE3;
    private PathChain INTAKE3;
    private PathChain INTAKE3_TO_SHOOT3;

    @Override
    public void init() {
        robot = new RobotHardware(this);
        follower = robot.getFollower();

        // Pose definitions (convert degrees → radians)
        // PP uses coordinates matching the visualizer
        Pose startPose  = new Pose(27.5, 129, Math.toRadians(90));
        Pose shootPose  = new Pose(59.5, 84, Math.toRadians(132));
        Pose intake1Pose= new Pose(16, 84, Math.toRadians(0));    // tangential heading doesn’t matter here
        Pose align2Pose = new Pose(43, 60, Math.toRadians(180));
        Pose intake2Pose= new Pose(14, 60, Math.toRadians(0));
        Pose clearPose  = new Pose(16.8, 72.2, Math.toRadians(90));
        Pose intake3Pose= new Pose(17.2, 35.5, Math.toRadians(0));
        Pose shoot3Pose = new Pose(59.5, 13, Math.toRadians(112));

        // Tell follower where we start
        follower.setStartingPose(startPose);

        // Build all paths
        START_TO_SHOOT = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        INTAKE1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intake1Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intake1Pose.getHeading())
                .build();

        INTAKE1_TO_SHOOT = follower.pathBuilder()
                .addPath(new BezierLine(intake1Pose, shootPose))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), shootPose.getHeading())
                .build();

        SHOOT1_TO_INTAKE2_ALIGN = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, align2Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), align2Pose.getHeading())
                .build();

        INTAKE2 = follower.pathBuilder()
                .addPath(new BezierLine(align2Pose, intake2Pose))
                .setLinearHeadingInterpolation(align2Pose.getHeading(), intake2Pose.getHeading())
                .build();

        INTAKE2_TO_SHOOT = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, shootPose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), shootPose.getHeading())
                .build();

        MOVE_TO_CLEAR = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, clearPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), clearPose.getHeading())
                .build();

        CLEAR_TO_INTAKE3 = follower.pathBuilder()
                .addPath(new BezierLine(clearPose, intake3Pose))
                .setLinearHeadingInterpolation(clearPose.getHeading(), intake3Pose.getHeading())
                .build();

        INTAKE3 = follower.pathBuilder()
                .addPath(new BezierLine(intake3Pose, intake3Pose)) // zero move, used for tangential tangent
                .setLinearHeadingInterpolation(intake3Pose.getHeading(), intake3Pose.getHeading())
                .build();

        INTAKE3_TO_SHOOT3 = follower.pathBuilder()
                .addPath(new BezierLine(intake3Pose, shoot3Pose))
                .setLinearHeadingInterpolation(intake3Pose.getHeading(), shoot3Pose.getHeading())
                .build();
    }


    @Override
    public void start() {
        follower.update();
        pathTimer.resetTimer();
        state = 0;
    }

    @Override
    public void loop() {
        follower.update();

        switch(state) {

            case 0: // START -> SHOOT
                follower.followPath(START_TO_SHOOT);
                state++;
                break;

            case 1:
                if(!follower.isBusy()) {
                    // shoot preload
                    robot.shooter.setVelocity(robot.calculateRegression(robot.getDistance()));
                    pathTimer.resetTimer();
                    state++;
                }
                break;

            case 2:
                // wait 3 sec
                if(pathTimer.getElapsedTimeSeconds() > 3) {
                    robot.shooter.setVelocity(robot.calculateRegression(robot.getDistance()));
                    robot.intake.setPower(1);
                    follower.followPath(INTAKE1);
                    state++;
                }
                break;

            case 3:
                if(!follower.isBusy()) {
                    robot.intake.setPower(0);
                    follower.followPath(INTAKE1_TO_SHOOT);
                    state++;
                }
                break;

            case 4:
                if(!follower.isBusy()) {
                    robot.shooter.setVelocity(robot.calculateRegression(robot.getDistance()));
                    pathTimer.resetTimer();
                    state++;
                }
                break;

            case 5:
                if(pathTimer.getElapsedTimeSeconds() > 3) {
                    robot.shooter.setVelocity(robot.calculateRegression(robot.getDistance()));
                    follower.followPath(SHOOT1_TO_INTAKE2_ALIGN);
                    state++;
                }
                break;

            case 6:
                if(!follower.isBusy()) {
                    robot.intake.setPower(1);
                    follower.followPath(INTAKE2);
                    state++;
                }
                break;

            case 7:
                if(!follower.isBusy()) {
                    robot.intake.setPower(0);
                    follower.followPath(INTAKE2_TO_SHOOT);
                    state++;
                }
                break;

            case 8:
                if(!follower.isBusy()) {
                    robot.shooter.setVelocity(robot.calculateRegression(robot.getDistance()));
                    pathTimer.resetTimer();
                    state++;
                }
                break;

            case 9:
                if(pathTimer.getElapsedTimeSeconds() > 3) {
                    robot.shooter.setVelocity(robot.calculateRegression(robot.getDistance()));
                    follower.followPath(MOVE_TO_CLEAR);
                    state++;
                }
                break;

            case 10:
                if(!follower.isBusy()) {
                    follower.followPath(CLEAR_TO_INTAKE3);
                    state++;
                }
                break;

            case 11:
                if(!follower.isBusy()) {
                    robot.intake.setPower(1);
                    follower.followPath(INTAKE3);
                    state++;
                }
                break;

            case 12:
                if(!follower.isBusy()) {
                    robot.intake.setPower(0);
                    follower.followPath(INTAKE3_TO_SHOOT3);
                    state++;
                }
                break;

            case 13:
                if(!follower.isBusy()) {
                    robot.shooter.setVelocity(robot.calculateRegression(robot.getDistance()));
                    pathTimer.resetTimer();
                    state++;
                }
                break;

            case 14:
                if(pathTimer.getElapsedTimeSeconds() > 3) {
                    robot.shooter.setVelocity(0);
                    state++; // auto done
                }
                break;

            default:
                // done
                break;
        }
    }
}
