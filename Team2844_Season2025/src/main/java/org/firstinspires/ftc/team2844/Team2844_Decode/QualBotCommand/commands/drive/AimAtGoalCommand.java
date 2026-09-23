package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.commands.drive;

import com.vcs.valleylib.core.command.Command;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.RobotConstants;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.subsystems.DriveSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.subsystems.VisionSubsystem;

/**
 * The auto-side version of aiming: a single in-place turn that cancels the
 * Limelight's tx, replacing the old {@code .turn(Math.toRadians(-tx))} that every
 * auto ran before shooting.
 *
 * <p>Does nothing at all when there is no target, which is what the old
 * {@code if (getTx() != -999)} guard around each of those turns amounted to.
 */
public final class AimAtGoalCommand {

    private AimAtGoalCommand() {}

    /** Default: correct by exactly the measured tx. */
    public static Command create(DriveSubsystem drive, VisionSubsystem vision) {
        return create(drive, vision, 0.0, 2.0);
    }

    /**
     * @param offsetDegrees bias added to the correction; a couple of degrees of
     *                      lead was hand-tuned into some of the old shots
     * @param timeoutSeconds gives up rather than hanging the routine on a turn
     *                       the follower can't settle
     */
    public static Command create(DriveSubsystem drive,
                                 VisionSubsystem vision,
                                 double offsetDegrees,
                                 double timeoutSeconds) {
        return new TurnToHeadingCommand(
                drive,
                () -> drive.getHeadingRadians() - Math.toRadians(vision.getTx() + offsetDegrees),
                timeoutSeconds)
                .unless(() -> vision.getTx() == RobotConstants.NO_TARGET)
                .withName("AimAtGoal");
    }
}
