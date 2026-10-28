package org.firstinspires.ftc.team2844.commands.transfer;

import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.subsystem.Subsystem;

import org.firstinspires.ftc.team2844.subsystems.TransferSubsystem;

import java.util.Collections;
import java.util.Set;

public class TransferCmd implements Command {
    TransferSubsystem transfer;
    double speed;

    public TransferCmd(TransferSubsystem transfer){
        this.transfer = transfer;
        this.speed = 1.0;
    }
    public TransferCmd(TransferSubsystem transfer, double speed){
        this.transfer = transfer;
        this.speed = speed;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.singleton(transfer);
    }

    @Override
    public void execute() {
        transfer.transfer(speed);
    }

    @Override
    public boolean isFinished() {
        return transfer.full();
    }

    @Override
    public void end(boolean interrupted) {
        transfer.stop();
    }
}
