package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.teleop;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.command.Commands;
import com.vcs.valleylib.ftc.RobotContainer;
import com.vcs.valleylib.ftc.input.CommandGamepad;
import com.vcs.valleylib.ftc.input.Trigger;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.QualBotRobot;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.RobotConstants;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.commands.drive.AimAssistDriveCommand;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.commands.intake.ExtakeCommand;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.commands.intake.IntakeCommand;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.commands.shooter.IdleShooterCommand;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.commands.shooter.ShootCommand;

/**
 * Driver controls for the one-person teleop.
 *
 * <p>Same layout the drivers already know from {@code OnePersonDriveBase}:
 *
 * <table>
 *   <tr><td>left stick</td><td>translate (field centric)</td></tr>
 *   <tr><td>right stick x</td><td>turn</td></tr>
 *   <tr><td>left trigger</td><td>intake (locked out when full)</td></tr>
 *   <tr><td>right trigger</td><td>extake</td></tr>
 *   <tr><td>right bumper</td><td>shoot, with aim assist held on</td></tr>
 *   <tr><td>a</td><td>toggle aim assist on its own</td></tr>
 *   <tr><td>b</td><td>shoot at the last known solution instead of the live one</td></tr>
 *   <tr><td>dpad up / down</td><td>trim target velocity by +-0.5</td></tr>
 *   <tr><td>right stick button</td><td>toggle baby mode</td></tr>
 *   <tr><td>guide</td><td>zero the field-centric heading</td></tr>
 *   <tr><td>gamepad 2 a / b</td><td>release / hold the ball stop</td></tr>
 * </table>
 */
public class TeleOpContainer extends RobotContainer {

    private static final double TRIGGER_THRESHOLD = 0.1;

    protected final QualBotRobot robot;
    protected final CommandGamepad driver;
    protected final CommandGamepad operator;

    /** Held once rather than rebuilt per poll; the shot and aim assist share it. */
    private final Trigger shootButton;

    /** While held, the shot reuses the last good solution instead of the live one. */
    private final Trigger holdLastSolution;

    /** Latched by the {@code a} button; aim assist also runs while shooting. */
    private boolean aimAssistEnabled = false;

    /**
     * Persistent dpad offset on the target velocity.
     *
     * <p>The old teleop recomputed the target from the distance regression every
     * loop, so its dpad adjustment was overwritten before it could ever take
     * effect. Keeping the trim separate from the solution makes the buttons
     * actually do what they look like they do.
     */
    private double velocityTrim = 0.0;

    public TeleOpContainer(QualBotRobot robot, Gamepad gamepad1, Gamepad gamepad2) {
        this.robot = robot;
        this.driver = CommandGamepad.forLogitechF310(gamepad1);
        this.operator = CommandGamepad.forLogitechF310(gamepad2);
        this.shootButton = driver.rightBumper();
        this.holdLastSolution = driver.b();

        configureBindings();
    }

    @Override
    public void configureBindings() {
        configureDefaults();
        configureDriving();
        configureIntake();
        configureShooting();
        configureOperator();
    }

    private void configureDefaults() {
        robot.drive.setDefaultCommand(robot.drive.run(
                () -> robot.drive.driveFieldCentric(forward(), strafe(), turn())));

        robot.shooter.setDefaultCommand(new IdleShooterCommand(robot.shooter));
    }

    private void configureDriving() {
        // These only flip a flag, so they deliberately take no subsystem
        // requirement -- claiming the drivetrain would blip the drive command.
        driver.rightStickButton().onTrue(
                Commands.runOnce(robot.drive::toggleBabyMode));

        driver.guide().onTrue(
                Commands.runOnce(robot.drive::resetHeading));

        driver.a().onTrue(
                Commands.runOnce(() -> aimAssistEnabled = !aimAssistEnabled));

        // Aim assist runs while it is latched on, and unconditionally while
        // shooting, so the shot is always taken at a corrected heading.
        new Trigger(() -> aimAssistEnabled).or(shootButton)
                .whileTrue(new AimAssistDriveCommand(
                        robot.drive, robot.vision, this::forward, this::strafe, this::turn));
    }

    private void configureIntake() {
        // Both are gated on not shooting: the shot drives the same roller to
        // feed, and without the gate the two whileTrue bindings would cancel and
        // reschedule each other every loop.
        driver.leftTriggerButton(TRIGGER_THRESHOLD).and(shootButton.negate()).whileTrue(
                new IntakeCommand(robot.intake, driver::leftTrigger));

        driver.rightTriggerButton(TRIGGER_THRESHOLD).and(shootButton.negate()).whileTrue(
                new ExtakeCommand(robot.intake, driver::rightTrigger));
    }

    private void configureShooting() {
        shootButton.whileTrue(
                ShootCommand.teleop(robot.shooter, robot.intake, robot.vision, this::targetVelocity));

        driver.dpadUp().onTrue(
                Commands.runOnce(() -> velocityTrim += RobotConstants.VELOCITY_TRIM_STEP));

        driver.dpadDown().onTrue(
                Commands.runOnce(() -> velocityTrim -= RobotConstants.VELOCITY_TRIM_STEP));

    }

    /**
     * Second-driver controls. Overridden by the tuning opmodes, which need the
     * whole second gamepad for themselves.
     */
    protected void configureOperator() {
        // The ball stop is its own servo, so nudging it never needs to take the
        // intake away from whatever is running.
        operator.a().onTrue(Commands.runOnce(robot.intake::releaseBall));
        operator.b().onTrue(Commands.runOnce(robot.intake::holdBall));
    }

    /* ===================== Stick shaping ===================== */

    /** +1 drives away from the driver wall. FTC sticks are negative-up. */
    private double forward() {
        return -driver.leftY();
    }

    /** +1 drives left. FTC sticks are positive-right. */
    private double strafe() {
        return -driver.leftX();
    }

    /** +1 turns counter-clockwise. */
    private double turn() {
        return -driver.rightX() * RobotConstants.ROT_CORRECTION;
    }

    /* ===================== Shot solution ===================== */

    /**
     * Target flywheel velocity: from the live distance normally, or frozen at
     * the last good solution while {@code b} is held — for shots the camera
     * can't see, or when the driver knows better than the tag.
     */
    public double targetVelocity() {
        double base = holdLastSolution.getAsBoolean()
                ? robot.shooter.getLastKnownVelocity()
                : robot.shooter.velocityForDistance(robot.vision.getDistanceInches());
        return base + velocityTrim;
    }

    public double getVelocityTrim() {
        return velocityTrim;
    }

    public boolean isAimAssistEnabled() {
        return aimAssistEnabled;
    }

    @Override
    public Command getAutonomousCommand() {
        // Autos are their own opmodes; nothing to run from teleop.
        return Commands.none();
    }
}
