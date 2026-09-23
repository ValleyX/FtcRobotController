package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandGroupBase;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.MoveArtifactClockwiseCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.NextSlotCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.SlotCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SpindexerSubsystem;

public class IntakeSortCmd extends SequentialCommandGroup {
//    private IntakeSubsystem intakeSubsystem;
//    private SpindexerSubsystem spindexerSubsystem;
//    private KickSubsystem kickSubsystem;

    public IntakeSortCmd(IntakeSubsystem intakeSubsystem, SpindexerSubsystem spindexerSubsystem, KickSubsystem kickSubsystem){
        addCommands(
                new ConditionalCommand(
                        new StopIntakeCmd(intakeSubsystem),
                        new ConditionalCommand(
                                new SequentialCommandGroup(new StopIntakeCmd(intakeSubsystem), new MoveArtifactClockwiseCmd(spindexerSubsystem, kickSubsystem).withTimeout(2000)),
                                new ActivateIntakeCmd(intakeSubsystem),
                                spindexerSubsystem::ballInBayOne
                        ),
                        spindexerSubsystem::fullSpindexer
                )
        );
    }








    /*NextSlotCmd slotCmd;


    public IntakeSortCmd(IntakeSubsystem intakeSubsystem, SpindexerSubsystem spindexerSubsystem, KickSubsystem kickSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        this.spindexerSubsystem = spindexerSubsystem;
        this.kickSubsystem = kickSubsystem;
        addRequirements(intakeSubsystem, spindexerSubsystem, kickSubsystem);
    }

    @Override
    public void initialize() {
        slotCmd = new NextSlotCmd(spindexerSubsystem, kickSubsystem);

        if(spindexerSubsystem.ballInBayOne() && (!spindexerSubsystem.ballInBayThree() || !spindexerSubsystem.ballInBayTwo()) && !slotCmd.isScheduled()){
            slotCmd.schedule();
        }
    }

    @Override
    public void execute(){
        if(!spindexerSubsystem.fullSpindexer() && !spindexerSubsystem.ballInBayOne()) {
            intakeSubsystem.activate(Constants.INTAKE_SPEED);
        } else {
            intakeSubsystem.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return slotCmd.isFinished() || spindexerSubsystem.fullSpindexer();
    }*/
}
