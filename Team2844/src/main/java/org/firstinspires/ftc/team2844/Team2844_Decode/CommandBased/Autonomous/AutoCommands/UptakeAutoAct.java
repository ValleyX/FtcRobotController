package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.AutoCommands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.TransferCommands.UptakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.TransferCommands.UptakeExtraCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;

import java.util.function.BooleanSupplier;

public class UptakeAutoAct implements Action {
    KickSubsystem kickSubsystem;
    BooleanSupplier topBroken;

    public UptakeAutoAct(KickSubsystem kickSubsystem, BooleanSupplier topBroken){
        this.kickSubsystem = kickSubsystem;
        this.topBroken = topBroken;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if(topBroken.getAsBoolean()){
            new CommandAction(new UptakeExtraCmd(kickSubsystem)).run(telemetryPacket);
        } else {
            new CommandAction(new UptakeCmd(kickSubsystem)).run(telemetryPacket);
        }

        return true;
    }
}
