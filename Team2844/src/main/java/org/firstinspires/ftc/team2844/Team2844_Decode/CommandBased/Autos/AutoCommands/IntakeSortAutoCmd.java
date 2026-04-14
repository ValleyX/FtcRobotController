package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autos.AutoCommands;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.SlotCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SpindexerSubsystem;

import java.util.concurrent.TimeUnit;

public class IntakeSortAutoCmd extends CommandBase {
    private IntakeSubsystem intakeSubsystem;
    private SpindexerSubsystem spindexerSubsystem;
    private KickSubsystem kickSubsystem;

    boolean ballInOne;
    boolean ballInTwo;
    boolean ballInThree;

    boolean full;

    SlotCmd slotCmd;

    ElapsedTime timer;
    double ballTimer;
    double ballTimerOffset;

    public IntakeSortAutoCmd(IntakeSubsystem intakeSubsystem, SpindexerSubsystem spindexerSubsystem, KickSubsystem kickSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        this.spindexerSubsystem = spindexerSubsystem;
        this.kickSubsystem = kickSubsystem;
        addRequirements(intakeSubsystem);

        timer = new ElapsedTime();
    }

    @Override
    public void initialize() {
        timer.reset();
        ballTimerOffset = 0.0;
        slotCmd = new SlotCmd(spindexerSubsystem, kickSubsystem, spindexerSubsystem.getSlot()+1);
    }

    @Override
    public void execute(){
        ballInOne = spindexerSubsystem.ballInBayOne();
        ballInTwo = spindexerSubsystem.ballInBayTwo();
        ballInThree = spindexerSubsystem.ballInBayThree();
        full = spindexerSubsystem.fullSpindexer();


        if(!full) {
            intakeSubsystem.activate(Constants.INTAKE_SPEED);
        } else {
            intakeSubsystem.stop();
        }

        if(ballInOne && (!ballInTwo || !ballInThree) && !slotCmd.isScheduled()){
            ballTimerOffset = timer.time(TimeUnit.MILLISECONDS);
            slotCmd.schedule();
        }

        if(intakeSubsystem.ballInBeam()){
            ballTimerOffset = timer.time(TimeUnit.MILLISECONDS);
        }

        ballTimer = timer.time(TimeUnit.MILLISECONDS) - ballTimerOffset;
    }

    @Override
    public boolean isFinished() {
        return (full || (ballTimer > 500 && timer.time(TimeUnit.MILLISECONDS) > 1500));
    }
}
