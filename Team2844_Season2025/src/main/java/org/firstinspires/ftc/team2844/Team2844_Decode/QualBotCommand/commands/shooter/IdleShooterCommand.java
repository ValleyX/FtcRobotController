package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.commands.shooter;

import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.subsystem.Subsystem;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.subsystems.ShooterSubsystem;

import java.util.Collections;
import java.util.Set;

/**
 * The shooter's default command: hold the flywheel at idle speed with the gate
 * shut.
 *
 * <p>Keeping the wheel turning between shots means the next shot only has to
 * make up the difference instead of spinning up from a dead stop. This runs
 * whenever nothing else has claimed the shooter, which replaces the old
 * "if not shooting and not intaking, set velocity 20" branch at the bottom of
 * the teleop loop.
 */
public class IdleShooterCommand implements Command {

    private final ShooterSubsystem shooter;

    public IdleShooterCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        shooter.closeGate();
        shooter.stowHood();
    }

    @Override
    public void execute() {
        shooter.holdIdleSpeed();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.singleton(shooter);
    }

    @Override
    public String getName() {
        return "IdleShooter";
    }
}
