package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.teleop;

import com.vcs.valleylib.ftc.opmode.CommandOpMode;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.PedroConstants;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.QualBotRobot;

/**
 * Shared teleop body. The alliance opmodes below only pick a Limelight pipeline.
 *
 * <p>All the per-loop work that used to fill {@code OnePersonDriveBase}'s while
 * loop now lives in subsystems and commands; this is only wiring and telemetry.
 */
public abstract class QualBotTeleOpBase extends CommandOpMode {

    protected QualBotRobot robot;
    protected TeleOpContainer container;

    /** 0 for blue, 1 for red. */
    protected abstract int limelightPipeline();

    @Override
    protected void initialize() {
        // Teleop starts wherever auto left off, which the follower has no way to
        // know, so the pose starts at the origin and the driver zeroes heading
        // with the guide button if auto ended facing an odd direction.
        robot = new QualBotRobot(hardwareMap, PedroConstants.ftcPose(0, 0, 0), limelightPipeline());
        container = new TeleOpContainer(robot, gamepad1, gamepad2);
    }

    @Override
    protected void onStart() {
        robot.drive.startTeleOp();
        robot.light.startMatchTimer();
        robot.intake.holdBall();
    }

    @Override
    protected void run() {
        telemetryBus.put("Drive fault", robot.drive.hasFault()
                ? robot.drive.getFaultReason()
                : "none");
        telemetryBus.put("Baby mode", robot.drive.isBabyMode());
        telemetryBus.put("Aim assist", container.isAimAssistEnabled());

        telemetryBus.put("Balls: one", robot.intake.hasBall());
        telemetryBus.put("Balls: two", robot.intake.hasTwoBalls());
        telemetryBus.put("Balls: full", robot.intake.isFull());

        telemetryBus.put("Shooter velocity", robot.shooter.getVelocity());
        telemetryBus.put("Shooter target", container.targetVelocity());
        telemetryBus.put("Shooter trim", container.getVelocityTrim());
        telemetryBus.put("Shooter power", robot.shooter.getPower());
        telemetryBus.put("Gate open", robot.shooter.isGateOpen());

        telemetryBus.put("Limelight tx", robot.vision.getTx());
        telemetryBus.put("Tag distance (in)", robot.vision.getDistanceInches());

        telemetryBus.put("Heading (deg)", robot.drive.getHeadingDegrees());
        telemetryBus.put("Pose X", robot.drive.getPose().getX());
        telemetryBus.put("Pose Y", robot.drive.getPose().getY());

        telemetryBus.put("Battery (V)", robot.getBatteryVoltage());
    }

    @Override
    protected boolean enableCommandLogging() {
        return true;
    }
}
