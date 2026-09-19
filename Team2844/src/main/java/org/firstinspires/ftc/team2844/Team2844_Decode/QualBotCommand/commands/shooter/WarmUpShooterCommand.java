package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.commands.shooter;

import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.subsystem.Subsystem;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.RobotConstants;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.subsystems.ShooterSubsystem;

import java.util.Collections;
import java.util.Set;

/**
 * Holds a fixed open-loop power on the flywheel.
 *
 * <p>Autos run this while driving to a shooting position so the wheel is already
 * moving when the shot command takes over — the old
 * {@code shooterHardware.setShootPower(0.2)} sprinkled between trajectories.
 *
 * <p>Never finishes on its own; run it in parallel with a path and let the path
 * end the group.
 */
public class WarmUpShooterCommand implements Command {

    private final ShooterSubsystem shooter;
    private final double power;

    public WarmUpShooterCommand(ShooterSubsystem shooter) {
        this(shooter, RobotConstants.AUTO_WARMUP_POWER);
    }

    public WarmUpShooterCommand(ShooterSubsystem shooter, double power) {
        this.shooter = shooter;
        this.power = power;
    }

    @Override
    public void initialize() {
        shooter.closeGate();
    }

    @Override
    public void execute() {
        shooter.setPower(power);
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
        return "WarmUpShooter";
    }
}
