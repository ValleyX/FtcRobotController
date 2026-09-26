package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.autos;

import com.pedropathing.geometry.Pose;
import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.time.RobotClock;
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

    /**
     * Autonomous is 30 seconds. Nothing this OpMode does should still be
     * commanding motors after that, whether because a path never converged or
     * because the OpMode was left running on the field.
     */
    private static final double AUTO_TIMEOUT_SECONDS = 30.0;

    /**
     * Ceiling on follower power during auto.
     *
     * <p>Barely changes path following, but it means a drivetrain that ends up
     * pushing against something is doing it at 85% rather than 100% while the
     * watchdog notices.
     */
    private static final double AUTO_MAX_POWER = 0.85;

    protected QualBotRobot robot;
    private Command routine;

    private double startSeconds = 0.0;
    private boolean timedOut = false;

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
        robot.drive.getFollower().setMaxPower(AUTO_MAX_POWER);

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
        startSeconds = RobotClock.seconds();
        timedOut = false;
        scheduler.schedule(routine);
    }

    @Override
    protected void run() {
        enforceTimeout();

        telemetryBus.put("Drive fault", robot.drive.hasFault()
                ? robot.drive.getFaultReason()
                : "none");
        telemetryBus.put("Timed out", timedOut);

        telemetryBus.put("Pose X", robot.drive.getPose().getX());
        telemetryBus.put("Pose Y", robot.drive.getPose().getY());
        telemetryBus.put("Heading (deg)", robot.drive.getHeadingDegrees());
        telemetryBus.put("Following", robot.drive.getFollower().isBusy());

        telemetryBus.put("Balls: one", robot.intake.hasBall());
        telemetryBus.put("Balls: full", robot.intake.isFull());
        telemetryBus.put("Shooter velocity", robot.shooter.getVelocity());
        telemetryBus.put("Limelight tx", robot.vision.getTx());
    }

    /**
     * Ends the routine and parks the drivetrain once the match period is up.
     *
     * <p>Cancelling is not enough on its own — the follower lives in the drive
     * subsystem and keeps driving the last path it was given regardless of which
     * command is scheduled — so the drivetrain is stopped explicitly.
     */
    private void enforceTimeout() {
        if (timedOut || RobotClock.seconds() - startSeconds < AUTO_TIMEOUT_SECONDS) {
            return;
        }
        timedOut = true;
        scheduler.cancelAll();
        robot.drive.stop();
        robot.intake.stop();
        robot.shooter.stop();
    }

    @Override
    protected boolean enableCommandLogging() {
        return true;
    }
}
