package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.commands.intake;

import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.subsystem.Subsystem;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.subsystems.IntakeSubsystem;

import java.util.Collections;
import java.util.Set;
import java.util.function.DoubleSupplier;

/**
 * Runs the intake roller at the power the driver is asking for, and refuses to
 * run once the robot is already holding three balls.
 */
public class IntakeCommand implements Command {

    private final IntakeSubsystem intake;
    private final DoubleSupplier power;
    private final boolean stopWhenFull;

    public IntakeCommand(IntakeSubsystem intake, DoubleSupplier power) {
        this(intake, power, true);
    }

    public IntakeCommand(IntakeSubsystem intake, DoubleSupplier power, boolean stopWhenFull) {
        this.intake = intake;
        this.power = power;
        this.stopWhenFull = stopWhenFull;
    }

    @Override
    public void execute() {
        if (stopWhenFull && intake.isFull()) {
            intake.stop();
        } else {
            intake.intake(power.getAsDouble());
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.singleton(intake);
    }

    @Override
    public String getName() {
        return "Intake";
    }
}
