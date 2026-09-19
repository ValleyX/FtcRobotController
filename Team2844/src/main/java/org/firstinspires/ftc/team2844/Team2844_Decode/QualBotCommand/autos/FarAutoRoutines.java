package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.command.Commands;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.QualBotRobot;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.commands.drive.AimAtGoalCommand;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.commands.intake.IntakeCommand;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.commands.shooter.ShootCommand;

import static org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.PedroConstants.ftcPose;

/**
 * The far-goal autonomouses.
 *
 * <p>All three start against the far wall, nose along it, turn onto the goal and
 * shoot. The blue six-ball variant then runs the ball line beside the wall and
 * comes back for a second shot; the two three-ball variants just clear the
 * starting area.
 *
 * <p>The three start poses are not mirror images of each other — each side was
 * lined up by hand — so each routine spells out its own.
 */
public final class FarAutoRoutines {

    private static final double AIM_TIMEOUT = 2.0;

    private FarAutoRoutines() {}

    /* ===================== Blue, three ball ===================== */

    public static final Pose BLUE_START = ftcPose(-63.5, 17, 0);
    private static final Pose BLUE_SHOOT = ftcPose(-60, 17, Math.toRadians(25));
    private static final Pose BLUE_PARK = ftcPose(-63.5, 63, Math.toRadians(-90));

    public static Command blueThreeBall(QualBotRobot robot) {
        Follower follower = robot.drive.getFollower();

        PathChain toShoot = AutoPaths.line(follower, BLUE_START, BLUE_SHOOT);
        PathChain park = AutoPaths.line(follower, BLUE_SHOOT, BLUE_PARK);

        return Commands.sequence(
                robot.drive.follow(toShoot),
                AimAtGoalCommand.create(robot.drive, robot.vision, 0.0, AIM_TIMEOUT),
                ShootCommand.autoFromFarGoal(robot.shooter, robot.intake, robot.vision),
                robot.drive.follow(park)
        ).withName("BlueFarThreeBall");
    }

    /* ===================== Blue, six ball ===================== */

    private static final Pose BLUE_PICKUP_END = ftcPose(-63.5, 63, Math.toRadians(90));
    private static final Pose BLUE_SHOOT_2 = ftcPose(-60, 12, Math.toRadians(25));
    private static final Pose BLUE_EXIT = ftcPose(-60, 36, 0);

    public static Command blueSixBall(QualBotRobot robot) {
        Follower follower = robot.drive.getFollower();

        PathChain toShoot = AutoPaths.line(follower, BLUE_START, BLUE_SHOOT);
        PathChain toPickup = AutoPaths.line(follower, BLUE_SHOOT, BLUE_PICKUP_END);
        PathChain backToShoot = AutoPaths.line(follower, BLUE_PICKUP_END, BLUE_SHOOT_2);
        PathChain exit = AutoPaths.line(follower, BLUE_SHOOT_2, BLUE_EXIT);

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

    public static final Pose RED_START = ftcPose(-63.25, -8.75, 0);
    private static final Pose RED_SHOOT = ftcPose(-60, -8.75, Math.toRadians(-25));
    private static final Pose RED_PARK = ftcPose(-60, -34, Math.toRadians(-90));

    public static Command redThreeBall(QualBotRobot robot) {
        Follower follower = robot.drive.getFollower();

        PathChain toShoot = AutoPaths.line(follower, RED_START, RED_SHOOT);
        PathChain park = AutoPaths.line(follower, RED_SHOOT, RED_PARK);

        return Commands.sequence(
                robot.drive.follow(toShoot),
                AimAtGoalCommand.create(robot.drive, robot.vision, 0.0, AIM_TIMEOUT),
                ShootCommand.autoFromFarGoal(robot.shooter, robot.intake, robot.vision),
                robot.drive.follow(park)
        ).withName("RedFarThreeBall");
    }
}
