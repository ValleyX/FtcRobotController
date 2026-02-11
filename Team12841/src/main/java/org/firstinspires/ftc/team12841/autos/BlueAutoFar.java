package org.firstinspires.ftc.team12841.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.team12841.RobotHardware;

@Autonomous(name="Blue Far", group="AUTO")
public class BlueAutoFar extends OpMode {

    private Follower follower;
    private RobotHardware robot;
    private Timer pathTimer;

    private int state = 0;

    // ---------------- DRIVE SPEEDS ----------------
    private static final double NORMAL_DRIVE_POWER = 1.0;
    private static final double INTAKE_DRIVE_POWER = 0.45;

    // ---------------- PATHS ----------------
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

    // ---------------- INTAKE STATE ----------------
    private boolean runIntake = false;
    private boolean intakeCaptured = false;

    private boolean aligning = false;
    // ---------------- FLICK CONTROL ----------------

    private void shootFlick(double power) {
        if (!runIntake) {
            robot.intake.setPower(1);
            robot.flick.setPower(power);
        }
    }

    private void intakeFlick(double power) {
        robot.flick.setPower(power);
    }

    // ---------------- INTAKE LOGIC ----------------

    private void runIntakeForward() {
        boolean beamNotBroken = robot.isBroken(); // ACTIVE LOW SENSOR
        boolean beamBroken = !beamNotBroken;

        robot.intake.setPower(1);

        if (!intakeCaptured) {
            if (!beamBroken) {
                intakeFlick(-1); // pull note in
            } else {
                intakeCaptured = true;
                intakeFlick(0);  // latch stop
            }
        } else {
            intakeFlick(0);
        }
    }

    private void align()
    {
        robot.alignWithLimelight(-1);
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
        robot.limelight.pipelineSwitch(1);

        Pose startPose = new Pose(0, 0, 0);
        Pose shootPreloadPose = new Pose(12.9639, 2.9152, 0.3621);

        Pose alignIntake1 = new Pose(31.1852, 0.3967, 1.6367);
        Pose intake1Pose = new Pose(29.7206, 44.379, 1.6285);
        Pose shoot1Pose = new Pose(1.2634, 8.6073, 0.2979);
        Pose parkPose = new Pose(30.04086, 7.5512, 1.5033);

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

        /*SHOOT_TO_ALIGN2 = follower.pathBuilder()
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
                .build();*/

        SHOOT_FAR_TO_PARK = follower.pathBuilder()
                .addPath(new BezierLine(shoot1Pose, parkPose))
                .setLinearHeadingInterpolation(shoot1Pose.getHeading(), parkPose.getHeading())
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
                robot.setShooterRPM(robot.calculateRegression(robot.getDistance()));
                follower.followPath(START_TO_SHOOT);
                pathTimer.resetTimer();
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
                if (pathTimer.getElapsedTimeSeconds() > 1.5) shootFlick(-1);
                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    aligning = false;
                    shootFlick(0);
                    intakeCaptured = false;
                    follower.followPath(SHOOT_TO_ALIGN1);
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
                    stopIntake();
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    follower.followPath(INTAKE1_TO_SHOOT);
                    state++;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    robot.setShooterRPM(robot.calculateRegression(robot.getDistance()));
                    aligning = true;
                    pathTimer.resetTimer();
                    state++;
                }
                break;

            case 6:
                if (pathTimer.getElapsedTimeSeconds() > 0.8) shootFlick(-1);
                if (pathTimer.getElapsedTimeSeconds() > 1.8) {
                    shootFlick(0);
                    aligning = false;
                    intakeCaptured = false;
                    follower.followPath(SHOOT_FAR_TO_PARK);
                    state++;
                }
                break;

           /* case 7:
                runIntake = true;
                follower.setMaxPower(INTAKE_DRIVE_POWER);
                if (!follower.isBusy()) {
                    follower.followPath(ALIGN2_TO_INTAKE2);
                    state++;
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    stopIntake();
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    follower.followPath(INTAKE2_TO_SHOOT);
                    state++;
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    robot.shooter.setVelocity(2200);
                    robot.alignWithLimelight(-1);
                    pathTimer.resetTimer();
                    state++;
                }
                break;

            case 10:
                if (pathTimer.getElapsedTimeSeconds() > 0.8) shootFlick(-1);
                if (pathTimer.getElapsedTimeSeconds() > 1.8) {
                    shootFlick(0);
                    intakeCaptured = false;
                    follower.followPath(SHOOT_TO_ALIGN3);
                    state++;
                }
                break;

            case 11:
                runIntake = true;
                follower.setMaxPower(INTAKE_DRIVE_POWER);
                if (!follower.isBusy()) {
                    follower.followPath(ALIGN3_TO_INTAKE3);
                    state++;
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    stopIntake();
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    follower.followPath(INTAKE3_TO_SHOOT_FAR);
                    state++;
                }
                break;*/

            /*case 7: //13
                if (!follower.isBusy()) {
                    robot.shooter.setVelocity(3700);
                    robot.alignWithLimelight(-1);
                    pathTimer.resetTimer();
                    state++;
                }
                break;

            case 8: //14
                if (pathTimer.getElapsedTimeSeconds() > 1) shootFlick(-1);
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    shootFlick(0);
                    follower.followPath(SHOOT_FAR_TO_PARK);
                    state++;
                }
                break;*/

            default:
                stopIntake();
                robot.shooter.setVelocity(0);
                break;
        }
        telemetry.addData("RPM", robot.shooter.getVelocity());
        telemetry.update();

        if (aligning) align();

        if (runIntake) runIntakeForward();
    }
}
