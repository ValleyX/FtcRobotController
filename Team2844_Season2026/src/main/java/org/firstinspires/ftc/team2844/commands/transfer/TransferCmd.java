package org.firstinspires.ftc.team2844.commands.transfer;

import com.vcs.valleylib.core.command.Command;

import org.firstinspires.ftc.team2844.subsystems.TransferSubsystem;

public class TransferCmd implements Command {
    TransferSubsystem transfer;

    public TransferCmd(TransferSubsystem transfer){
        this.transfer = transfer;
    }

    @Override
    public void execute() {
        transfer.transfer();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
