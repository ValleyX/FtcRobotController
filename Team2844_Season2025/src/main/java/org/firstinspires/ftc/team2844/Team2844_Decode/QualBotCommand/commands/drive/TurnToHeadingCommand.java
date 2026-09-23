package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.commands.drive;

import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.subsystem.Subsystem;
import com.vcs.valleylib.core.time.RobotClock;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.subsystems.DriveSubsystem;

import java.util.Collections;
import java.util.Set;
import java.util.function.DoubleSupplier;

/**
 * Turns in place to an absolute field heading using the follower's turn
 * controller — the command-based stand-in for Road Runner's {@code .turnTo()}.
 *
 * <p>The heading is a supplier rather than a value so callers can decide it at
 * the moment the command starts (for example, from a live Limelight reading)
 * instead of when the routine is assembled.
 */
public class TurnToHeadingCommand implements Command {

    private final DriveSubsystem drive;
    private final DoubleSupplier headingRadians;
    private final double timeoutSeconds;

    private double startSeconds;

    public TurnToHeadingCommand(DriveSubsystem drive, DoubleSupplier headingRadians, double timeoutSeconds) {
        this.drive = drive;
        this.headingRadians = headingRadians;
        this.timeoutSeconds = timeoutSeconds;
    }

    public TurnToHeadingCommand(DriveSubsystem drive, double headingRadians, double timeoutSeconds) {
        this(drive, () -> headingRadians, timeoutSeconds);
    }

    @Override
    public void initialize() {
        startSeconds = RobotClock.seconds();
        drive.turnTo(headingRadians.getAsDouble());
    }

    @Override
    public void execute() {
        // The follower does the work in its own update(); nothing to do per loop.
    }

    @Override
    public boolean isFinished() {
        return !drive.isTurning() || RobotClock.seconds() - startSeconds >= timeoutSeconds;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.singleton(drive);
    }

    @Override
    public String getName() {
        return "TurnToHeading";
    }
}
