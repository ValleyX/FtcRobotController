package org.firstinspires.ftc.team2844.commands.transfer;

import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.subsystem.Subsystem;

import org.firstinspires.ftc.team2844.subsystems.TransferSubsystem;

import java.util.Collections;
import java.util.Set;

public class CloseGatesCmd implements Command {

    TransferSubsystem transfer;

    public CloseGatesCmd(TransferSubsystem transfer){
        this.transfer = transfer;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.singleton(transfer);
    }

    @Override
    public void execute() {
        transfer.closeGates();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
