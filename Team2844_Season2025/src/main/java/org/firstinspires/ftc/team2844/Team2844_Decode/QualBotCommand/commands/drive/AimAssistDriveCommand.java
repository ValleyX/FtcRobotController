package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.commands.drive;

import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.subsystem.Subsystem;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.RobotConstants;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.subsystems.DriveSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.subsystems.VisionSubsystem;

import java.util.Collections;
import java.util.Set;
import java.util.function.DoubleSupplier;

/**
 * Teleop drive with the turn axis taken over by the Limelight.
 *
 * <p>The driver keeps translation; heading is closed onto the goal with a
 * proportional term on tx. Because this requires the drivetrain it preempts the
 * normal drive command for as long as it is scheduled, and hands control back
 * untouched when it ends.
 *
 * <p>When the Limelight loses the target the driver's own turn stick comes back,
 * so losing sight of the goal never locks the robot's heading.
 */
public class AimAssistDriveCommand implements Command {

    private final DriveSubsystem drive;
    private final VisionSubsystem vision;
    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;
    private final DoubleSupplier turn;

    public AimAssistDriveCommand(DriveSubsystem drive,
                                 VisionSubsystem vision,
                                 DoubleSupplier forward,
                                 DoubleSupplier strafe,
                                 DoubleSupplier turn) {
        this.drive = drive;
        this.vision = vision;
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
    }

    @Override
    public void execute() {
        double tx = vision.getTx();
        double turnOutput;

        if (tx == RobotConstants.NO_TARGET) {
            turnOutput = turn.getAsDouble() * RobotConstants.ROT_CORRECTION;
        } else if (Math.abs(tx) < RobotConstants.AIM_TOLERANCE_DEGREES) {
            turnOutput = 0.0;
        } else {
            // Positive tx means the goal is to the right, so turn clockwise:
            // negative in Pedro's counter-clockwise-positive convention.
            turnOutput = clamp(-RobotConstants.AIM_KP * tx);
        }

        drive.driveFieldCentric(forward.getAsDouble(), strafe.getAsDouble(), turnOutput);
    }

    private static double clamp(double power) {
        return Math.max(-RobotConstants.AIM_MAX_POWER, Math.min(RobotConstants.AIM_MAX_POWER, power));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.singleton(drive);
    }

    @Override
    public String getName() {
        return "AimAssistDrive";
    }
}
