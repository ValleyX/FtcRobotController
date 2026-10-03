package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.autos;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.command.Commands;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.QualBotRobot;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.commands.drive.AimAtGoalCommand;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.commands.intake.IntakeCommand;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.commands.shooter.ShootCommand;

import static org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.PedroConstants.FIELD;

/**
 * The far-goal autonomouses.
 *
 * <p>All three start against the far wall, nose along it, turn onto the goal and
 * shoot. The blue six-ball variant then runs the ball line beside the wall and
 * comes back for a second shot; the two three-ball variants just clear the
 * starting area.
 *
 * <p>The three start poses are not mirror images of each other -- each side was
 * lined up by hand -- so every pose here is written out in absolute field
 * coordinates and built through {@link org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.PedroConstants#FIELD},
 * the factory that applies no alliance transform. Sending the red poses through
 * the mirroring factory would flip numbers that are already correct.
 *
 * <p>Headings are in degrees: the factory converts, so there is no
 * {@code Math.toRadians} noise.
 */
public final class FarAutoRoutines {

    private static final double AIM_TIMEOUT = 2.0;

    private FarAutoRoutines() {}

    /* ===================== Blue, three ball ===================== */

    public static final Pose BLUE_START = FIELD.of(-63.5, 17, 0);
    private static final Pose BLUE_SHOOT = FIELD.of(-60, 17, 25);
    private static final Pose BLUE_PARK = FIELD.of(-63.5, 63, -90);

    public static Command blueThreeBall(QualBotRobot robot) {
        Path toShoot = AutoPaths.line(BLUE_START, BLUE_SHOOT);
        Path park = AutoPaths.line(BLUE_SHOOT, BLUE_PARK);

        return Commands.sequence(
                robot.drive.follow(toShoot),
                AimAtGoalCommand.create(robot.drive, robot.vision, 0.0, AIM_TIMEOUT),
                ShootCommand.autoFromFarGoal(robot.shooter, robot.intake, robot.vision),
                robot.drive.follow(park)
        ).withName("BlueFarThreeBall");
    }

    /* ===================== Blue, six ball ===================== */

    private static final Pose BLUE_PICKUP_END = FIELD.of(-63.5, 63, 90);
    private static final Pose BLUE_SHOOT_2 = FIELD.of(-60, 12, 25);
    private static final Pose BLUE_EXIT = FIELD.of(-60, 36, 0);

    public static Command blueSixBall(QualBotRobot robot) {
        Path toShoot = AutoPaths.line(BLUE_START, BLUE_SHOOT);
        Path toPickup = AutoPaths.line(BLUE_SHOOT, BLUE_PICKUP_END);
        Path backToShoot = AutoPaths.line(BLUE_PICKUP_END, BLUE_SHOOT_2);
        Path exit = AutoPaths.line(BLUE_SHOOT_2, BLUE_EXIT);

        // The roller runs for the whole trip up the ball line and stops when the
        // path does; it does not stop early on a full robot, matching the
        // original, which only checked the ball count after the run.
        Command collectAlongLine = robot.drive.follow(toPickup)
                .deadlineWith(new IntakeCommand(robot.intake, () -> 1.0, false));

        return Commands.sequence(
                robot.drive.follow(toShoot),
                AimAtGoalCommand.create(robot.drive, robot.vision, 0.0, AIM_TIMEOUT),
                ShootCommand.autoFromFarGoal(robot.shooter, robot.intake, robot.vision),

                collectAlongLine,
                robot.drive.follow(backToShoot),
                AimAtGoalCommand.create(robot.drive, robot.vision, 0.0, AIM_TIMEOUT),
                ShootCommand.autoFromFarGoal(robot.shooter, robot.intake, robot.vision),

                robot.drive.follow(exit)
        ).withName("BlueFarSixBall");
    }

    /* ===================== Red, three ball ===================== */

    public static final Pose RED_START = FIELD.of(-63.25, -8.75, 0);
    private static final Pose RED_SHOOT = FIELD.of(-60, -8.75, -25);
    private static final Pose RED_PARK = FIELD.of(-60, -34, -90);

    public static Command redThreeBall(QualBotRobot robot) {
        Path toShoot = AutoPaths.line(RED_START, RED_SHOOT);
        Path park = AutoPaths.line(RED_SHOOT, RED_PARK);

        return Commands.sequence(
                robot.drive.follow(toShoot),
                AimAtGoalCommand.create(robot.drive, robot.vision, 0.0, AIM_TIMEOUT),
                ShootCommand.autoFromFarGoal(robot.shooter, robot.intake, robot.vision),
                robot.drive.follow(park)
        ).withName("RedFarThreeBall");
    }
}
