package org.firstinspires.ftc.team2844.commands.shoot;

import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.subsystem.Subsystem;

import org.firstinspires.ftc.team2844.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.team2844.subsystems.TransferSubsystem;

import java.util.Set;

public class ShootCmd implements Command {

    ShooterSubsystem nectar, pollen;
    TransferSubsystem transfer;
    IntakeSubsystem intake;

    public ShootCmd(IntakeSubsystem intake, ShooterSubsystem nectar, ShooterSubsystem pollen, TransferSubsystem transfer){
        this.intake = intake;
        this.nectar = nectar;
        this.pollen = pollen;
        this.transfer = transfer;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(intake, nectar, pollen, transfer);
    }

    @Override
    public void initialize() {
        transfer.openGates();
    }

    @Override
    public void execute() {
        nectar.shoot();
        pollen.shoot();
        if(nectar.atVelocity() && pollen.atVelocity()){
            transfer.transfer();
            intake.intake();
        } else {
            transfer.stop();
            intake.stopIntake();
        }
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
        nectar.idleShoot();
        pollen.idleShoot();
    }
}
