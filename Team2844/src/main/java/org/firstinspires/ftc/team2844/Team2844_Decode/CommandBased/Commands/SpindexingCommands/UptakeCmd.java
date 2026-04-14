package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;

public class UptakeCmd extends CommandBase {
    KickSubsystem kickSubsystem;

    public UptakeCmd(KickSubsystem kickSubsystem){
        this.kickSubsystem = kickSubsystem;
        addRequirements(kickSubsystem);
    }

    @Override
    public void initialize() {
        kickSubsystem.rotateKickerDown();
        kickSubsystem.runKickerSpin();
        kickSubsystem.runSFeedForward();
    }


    @Override
    public boolean isFinished() {
        return true;
    }
}
