package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.AutoCommands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.IntakeLineCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterFeedSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SpindexerSubsystem;

public class IntakeLineAct implements Action {
    ShooterFeedSubsystem shooterFeedSubsystem;
    IntakeSubsystem intakeSubsystem;
    SpindexerSubsystem spindexerSubsystem;
    KickSubsystem kickSubsystem;
    ElapsedTime timer;
    boolean init;
    boolean startedFull;
    public IntakeLineAct(ShooterFeedSubsystem shooterFeedSubsystem, IntakeSubsystem intakeSubsystem, SpindexerSubsystem spindexerSubsystem, KickSubsystem kickSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.shooterFeedSubsystem = shooterFeedSubsystem;
        this.kickSubsystem = kickSubsystem;
        this.spindexerSubsystem = spindexerSubsystem;
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        init = true;
    }


    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if(init){
            startedFull = shooterFeedSubsystem.topBroken();
            intakeSubsystem.activate(Constants.INTAKE_SPEED);
            kickSubsystem.runKickerSpin();
            kickSubsystem.runSFeedForward();
            init = false;
        }

        boolean topBroken = shooterFeedSubsystem.topBroken();
        if (!topBroken) {
            kickSubsystem.rotateKickerDown();
            timer.reset();
        } else {
            if(timer.time() < 450 && !startedFull){
                kickSubsystem.rotateKickerDownIntake();
            } else {
                kickSubsystem.stopKickerSpin();
                kickSubsystem.runSFeedBackward();
            }
        }
        return true;
    }
}
