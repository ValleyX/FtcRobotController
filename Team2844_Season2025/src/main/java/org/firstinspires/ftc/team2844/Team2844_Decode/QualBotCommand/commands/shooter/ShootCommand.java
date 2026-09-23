package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.commands.shooter;

import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.subsystem.Subsystem;
import com.vcs.valleylib.core.time.RobotClock;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.RobotConstants;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.subsystems.VisionSubsystem;

import java.util.HashSet;
import java.util.Set;
import java.util.function.DoubleSupplier;

/**
 * Spins the flywheel to the solution for the current distance and feeds balls
 * into it as fast as the wheel can recover.
 *
 * <p>This is the loop that used to live inline in both the teleop right-bumper
 * branch and the {@code shoot()} helper each auto carried its own copy of:
 *
 * <ol>
 *   <li>lift the ball stop out of the way,</li>
 *   <li>hold the hood and the target velocity from the distance regressions,</li>
 *   <li>once the wheel reaches speed, hand off from the velocity PID to the
 *       open-loop hold power and open the gate,</li>
 *   <li>when a ball drags the wheel below tolerance, shut the gate and let it
 *       recover.</li>
 * </ol>
 *
 * <p>Requires the intake as well as the shooter, because feeding is the intake
 * roller's job — so an intake command and a shot can never fight over it.
 */
public class ShootCommand implements Command {

    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final VisionSubsystem vision;

    /**
     * Target velocity, or null to recompute it from the live distance every loop.
     * Teleop passes one in so the dpad trim and the "hold last shot" button work.
     */
    private final DoubleSupplier velocityOverride;

    /** Auto shots end themselves; the teleop shot runs until the button is let go. */
    private final boolean endWhenEmpty;

    /** Distance used when the Limelight sees nothing, or NO_TARGET to just idle. */
    private final double fallbackDistance;

    private double emptySinceSeconds = Double.NaN;

    public ShootCommand(ShooterSubsystem shooter,
                        IntakeSubsystem intake,
                        VisionSubsystem vision,
                        DoubleSupplier velocityOverride,
                        boolean endWhenEmpty,
                        double fallbackDistance) {
        this.shooter = shooter;
        this.intake = intake;
        this.vision = vision;
        this.velocityOverride = velocityOverride;
        this.endWhenEmpty = endWhenEmpty;
        this.fallbackDistance = fallbackDistance;
    }

    /** Teleop shot: driver-trimmed velocity, runs until the trigger is released. */
    public static ShootCommand teleop(ShooterSubsystem shooter,
                                      IntakeSubsystem intake,
                                      VisionSubsystem vision,
                                      DoubleSupplier velocity) {
        return new ShootCommand(shooter, intake, vision, velocity, false, RobotConstants.NO_TARGET);
    }

    /**
     * Auto shot: empties the robot, then stops.
     *
     * <p>With no tag in view it falls back to the default velocity and a stowed
     * hood, which is what the near-goal auto did — from that close, a blind shot
     * at the default is better than one aimed at a guessed distance.
     */
    public static ShootCommand auto(ShooterSubsystem shooter,
                                    IntakeSubsystem intake,
                                    VisionSubsystem vision) {
        return new ShootCommand(shooter, intake, vision, null, true, RobotConstants.NO_TARGET);
    }

    /**
     * Auto shot that assumes a distance when the tag is not visible.
     *
     * <p>The far-goal autos always shoot from roughly the same spot, so guessing
     * that distance beats falling back to a default that is tuned for point blank.
     */
    public static ShootCommand autoFromFarGoal(ShooterSubsystem shooter,
                                               IntakeSubsystem intake,
                                               VisionSubsystem vision) {
        return new ShootCommand(shooter, intake, vision, null, true,
                RobotConstants.FALLBACK_SHOT_DISTANCE);
    }

    @Override
    public void initialize() {
        emptySinceSeconds = Double.NaN;
        intake.releaseBall();
    }

    @Override
    public void execute() {
        double distance = vision.getDistanceInches();
        if (distance == RobotConstants.NO_TARGET) {
            distance = fallbackDistance;
        }

        double targetVelocity = velocityOverride != null
                ? velocityOverride.getAsDouble()
                : shooter.velocityForDistance(distance);

        shooter.setHood(shooter.hoodForDistance(distance));
        shooter.setVelocity(targetVelocity);

        if (intake.hasBall()) {
            emptySinceSeconds = Double.NaN;
            gateOnVelocity(targetVelocity);
        } else {
            if (Double.isNaN(emptySinceSeconds)) {
                emptySinceSeconds = RobotClock.seconds();
            }
            // The sensors are clear but the last ball is still on its way out, so
            // stop gating on velocity and just keep feeding for the buffer. The
            // old routines ended the same way, with an unconditional feed and sleep.
            shooter.openGate();
            intake.intake(1.0);
        }
    }

    /** Feeds only while the wheel is at speed, and pauses when a ball drags it down. */
    private void gateOnVelocity(double targetVelocity) {
        if (shooter.atVelocity(targetVelocity)) {
            // At speed: hand off from the velocity PID to the open-loop hold
            // power so a passing ball cannot make the PID lurch, then feed.
            shooter.setPower(shooter.holdPowerFor(targetVelocity));
            shooter.openGate();
            intake.intake(1.0);
        } else if (shooter.belowVelocity(targetVelocity)) {
            // A ball dragged the wheel down; stop feeding until it recovers.
            shooter.closeGate();
            intake.stop();
        }
    }

    @Override
    public boolean isFinished() {
        if (!endWhenEmpty || Double.isNaN(emptySinceSeconds)) {
            return false;
        }
        return RobotClock.seconds() - emptySinceSeconds >= RobotConstants.SHOOT_BUFFER_SECONDS;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        intake.holdBall();
        shooter.closeGate();
        shooter.stowHood();
        shooter.stop();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        Set<Subsystem> requirements = new HashSet<>();
        requirements.add(shooter);
        requirements.add(intake);
        return requirements;
    }

    @Override
    public String getName() {
        return "Shoot";
    }
}
