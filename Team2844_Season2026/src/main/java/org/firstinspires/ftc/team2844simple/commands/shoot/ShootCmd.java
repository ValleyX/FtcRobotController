package org.firstinspires.ftc.team2844simple.commands.shoot;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.subsystem.Subsystem;

import org.firstinspires.ftc.team2844simple.helpers.Constants;
import org.firstinspires.ftc.team2844simple.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844simple.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.team2844simple.subsystems.TransferSubsystem;

import java.util.Set;

public class ShootCmd implements Command {

    ShooterSubsystem pollen;
    TransferSubsystem transfer;
    IntakeSubsystem intake;
    double timeout;

    ElapsedTime time;

    public ShootCmd(IntakeSubsystem intake, ShooterSubsystem pollen, TransferSubsystem transfer, double timeout){
        this.intake = intake;
        this.pollen = pollen;
        this.transfer = transfer;
        this.timeout = timeout;
        time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(intake, pollen, transfer);
    }

    @Override
    public void initialize() {
        intake.stopIntake();
        transfer.stop();
        time.reset();
    }

    @Override
    public void execute() {
        pollen.shoot();
        if(pollen.atVelocity()){
            transfer.openTransferGates();
        } else {
            transfer.closeTransferGates();
        }

        if(time.time()>timeout/2.0) {
            intake.intake();
        }
        transfer.transfer();
    }

    @Override
    public boolean isFinished() {
        return time.time()>= timeout;
    }

    @Override
    public void end(boolean interrupted) {
        transfer.stop();
        intake.stopIntake();
        transfer.closeGates();
        pollen.idleShoot();
    }
}
