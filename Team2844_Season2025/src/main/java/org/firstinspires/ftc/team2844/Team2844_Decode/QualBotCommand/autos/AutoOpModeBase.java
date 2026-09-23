package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.autos;

import com.pedropathing.geometry.Pose;
import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.ftc.opmode.CommandOpMode;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.QualBotRobot;

/**
 * Shared body for every autonomous opmode.
 *
 * <p>The routine is built during init but only scheduled in {@code onStart()} —
 * scheduling a command runs its {@code initialize()} immediately, so building
 * and scheduling in one step during init would start the first path before the
 * driver pressed play.
 */
public abstract class AutoOpModeBase extends CommandOpMode {

    protected QualBotRobot robot;
    private Command routine;

    /** Where the robot is placed, in Pedro coordinates. */
    protected abstract Pose startingPose();

    /** 0 for blue, 1 for red. */
    protected abstract int limelightPipeline();

    /** The routine to run when the match starts. */
    protected abstract Command buildRoutine(QualBotRobot robot);

    @Override
    protected void initialize() {
        robot = new QualBotRobot(hardwareMap, startingPose(), limelightPipeline());

        // Auto has time to wait for a real spin-up, so hold the flywheel to a
        // tighter window than teleop does before letting a ball through.
        robot.shooter.useAutoVelocityThreshold();
        robot.intake.holdBall();

        routine = buildRoutine(robot);
    }

    @Override
    protected void initLoop() {
        telemetryBus.put("Routine", routine.getName());
        telemetryBus.put("Battery (V)", robot.getBatteryVoltage());
        telemetryBus.put("Tag visible", robot.vision.isTargetVisible());
        telemetryBus.put("Tag distance (in)", robot.vision.getDistanceInches());
    }

    @Override
    protected void onStart() {
        scheduler.schedule(routine);
    }

    @Override
    protected void run() {
        telemetryBus.put("Pose X", robot.drive.getPose().getX());
        telemetryBus.put("Pose Y", robot.drive.getPose().getY());
        telemetryBus.put("Heading (deg)", robot.drive.getHeadingDegrees());
        telemetryBus.put("Following", robot.drive.getFollower().isBusy());

        telemetryBus.put("Balls: one", robot.intake.hasBall());
        telemetryBus.put("Balls: full", robot.intake.isFull());
        telemetryBus.put("Shooter velocity", robot.shooter.getVelocity());
        telemetryBus.put("Limelight tx", robot.vision.getTx());
    }

    @Override
    protected boolean enableCommandLogging() {
        return true;
    }
}
