package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.command.Commands;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.QualBotRobot;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.commands.drive.AimAtGoalCommand;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.commands.drive.TurnToHeadingCommand;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.commands.intake.CollectCommand;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.commands.shooter.ShootCommand;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.commands.shooter.WarmUpShooterCommand;

import java.util.concurrent.atomic.AtomicBoolean;

/**
 * The near-goal autonomous, as one command.
 *
 * <p>Three shooting cycles: shoot the preload, run each of the two ball lines,
 * and shoot again after each. After a pickup the robot checks whether it
 * actually got three balls — if it did it drives straight to the shot, and if it
 * didn't it wiggles in place to shake the last ball in before taking a different
 * route to the shooting spot. That branch is the {@code shakeAndRecover} path
 * below, and it is the same logic the Road Runner version ran.
 *
 * <p>Both alliances run this; only {@link CloseAutoGeometry} differs.
 */
public final class CloseAutoRoutine {

    /** How far the shake turns each way, in degrees. */
    private static final double SHAKE_DEGREES = 15.0;

    private static final double SHAKE_TIMEOUT = 1.0;
    private static final double AIM_TIMEOUT = 2.0;

    /**
     * The first shot was tuned with a few degrees of lead on the tag.
     *
     * <p>Road Runner turned by {@code -tx + 6}, which is the same as aiming at
     * a tx target of -6.
     */
    private static final double SHOT_2_AIM_OFFSET_DEGREES = -6.0;

    private CloseAutoRoutine() {}

    public static Command build(QualBotRobot robot, CloseAutoGeometry geometry) {
        Follower follower = robot.drive.getFollower();

        /* ---------- Paths ---------- */

        PathChain toShoot1 = AutoPaths.line(follower, geometry.start, geometry.shoot1);

        PathChain pickup1 = AutoPaths.through(follower,
                geometry.shoot1, geometry.pickup1Entry, geometry.pickup1Far, geometry.pickup1Back);

        PathChain pickup1ToShoot2 = AutoPaths.curve(follower,
                geometry.pickup1Back,
                AutoPaths.bulgedControl(geometry.pickup1Back, geometry.shoot2, 8),
                geometry.shoot2);

        PathChain pickup2 = AutoPaths.through(follower,
                geometry.shoot2, geometry.pickup2Entry, geometry.pickup2Far, geometry.pickup2Back);

        PathChain pickup2ToShoot3 = AutoPaths.through(follower,
                geometry.pickup2Back, geometry.exit2, geometry.shoot3);

        /* ---------- Shared pieces ---------- */

        // Set on the first pickup that comes up short. The second cycle then
        // takes the direct route regardless of what its own sensors say, because
        // a short robot has nothing left to shake in.
        AtomicBoolean cameUpShort = new AtomicBoolean(false);

        Command driveWarm1 = robot.drive.follow(toShoot1)
                .deadlineWith(new WarmUpShooterCommand(robot.shooter));

        Command collectLine1 = robot.drive.follow(pickup1)
                .deadlineWith(new CollectCommand(robot.intake, robot.shooter, robot::isHighVoltage));

        Command collectLine2 = robot.drive.follow(pickup2)
                .deadlineWith(new CollectCommand(robot.intake, robot.shooter, robot::isHighVoltage));

        Command straightToShoot2 = robot.drive.follow(pickup1ToShoot2)
                .deadlineWith(new WarmUpShooterCommand(robot.shooter));

        Command shakeThenShoot2 = Commands.sequence(
                Commands.runOnce(() -> cameUpShort.set(true)),
                shake(robot),
                straightToShoot2);

        Command straightToShoot3 = robot.drive.follow(pickup2ToShoot3)
                .deadlineWith(new WarmUpShooterCommand(robot.shooter));

        Command shakeThenShoot3 = Commands.sequence(
                shake(robot),
                straightToShoot3);

        /* ---------- Routine ---------- */

        return Commands.sequence(
                driveWarm1,
                ShootCommand.auto(robot.shooter, robot.intake, robot.vision),

                collectLine1,
                Commands.either(straightToShoot2, shakeThenShoot2, robot.intake::isFull),
                AimAtGoalCommand.create(robot.drive, robot.vision, SHOT_2_AIM_OFFSET_DEGREES, AIM_TIMEOUT),
                ShootCommand.auto(robot.shooter, robot.intake, robot.vision),

                collectLine2,
                Commands.either(straightToShoot3, shakeThenShoot3,
                        () -> robot.intake.isFull() || cameUpShort.get()),
                AimAtGoalCommand.create(robot.drive, robot.vision),
                ShootCommand.auto(robot.shooter, robot.intake, robot.vision)
        ).withName("CloseAuto");
    }

    /**
     * Wiggle in place: turn one way, then back past centre. Shakes a ball that
     * is sitting on the lip of the intake the rest of the way in.
     */
    private static Command shake(QualBotRobot robot) {
        return Commands.sequence(
                new TurnToHeadingCommand(robot.drive,
                        () -> robot.drive.getHeadingRadians() - Math.toRadians(SHAKE_DEGREES),
                        SHAKE_TIMEOUT),
                new TurnToHeadingCommand(robot.drive,
                        () -> robot.drive.getHeadingRadians() + Math.toRadians(2 * SHAKE_DEGREES),
                        SHAKE_TIMEOUT));
    }
}
