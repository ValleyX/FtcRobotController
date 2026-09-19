package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.TransferCommands;

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
        kickSubsystem.rotateKickerDown(); //TANNER!!!!!! I have an UptakeExtraCmd, so if you want it to Uptake extra, use that
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
