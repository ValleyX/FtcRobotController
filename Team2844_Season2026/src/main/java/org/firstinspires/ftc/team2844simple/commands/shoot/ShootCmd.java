package org.firstinspires.ftc.team2844simple.commands.shoot;

import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.subsystem.Subsystem;

import org.firstinspires.ftc.team2844simple.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844simple.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.team2844simple.subsystems.TransferSubsystem;

import java.util.Set;

public class ShootCmd implements Command {

    ShooterSubsystem pollen;
    TransferSubsystem transfer;
    IntakeSubsystem intake;

    public ShootCmd(IntakeSubsystem intake, ShooterSubsystem pollen, TransferSubsystem transfer){
        this.intake = intake;
        this.pollen = pollen;
        this.transfer = transfer;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(intake, pollen, transfer);
    }

    @Override
    public void initialize() {
        intake.stopIntake();
        transfer.stop();
    }

    @Override
    public void execute() {
        pollen.shoot();
        if(pollen.atVelocity()){
            transfer.openPollenGate();
        } else {
            transfer.closePollenGate();
        }
        intake.intake();
        transfer.transfer();
    }

    @Override
    public boolean isFinished() {
        return intake.empty();
    }

    @Override
    public void end(boolean interrupted) {
        transfer.stop();
        intake.stopIntake();
        transfer.closeGates();
        pollen.idleShoot();
    }
}
