package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;

public class UptakeExtraCmd extends CommandBase {
    KickSubsystem kickSubsystem;

    public UptakeExtraCmd(KickSubsystem kickSubsystem){
        this.kickSubsystem = kickSubsystem;
        addRequirements(kickSubsystem);
    }

    @Override
    public void initialize() {
        kickSubsystem.rotateKickerDownExtra();
        kickSubsystem.runKickerSpin();
        kickSubsystem.runSFeedForward();
        // Extra kick logic? Yea that would probably go in here. or maybe in the shoot logic itself
        //spindexerSubsystem.runToShootSlot(spindexerSubsystem.getSlot());
    }


    @Override
    public boolean isFinished() {
        return true;
    }
}
