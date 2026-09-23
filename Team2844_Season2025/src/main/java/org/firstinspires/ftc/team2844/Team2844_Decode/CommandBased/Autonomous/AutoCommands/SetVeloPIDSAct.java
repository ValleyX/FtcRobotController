package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.AutoCommands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterSubsystem;

public class SetVeloPIDSAct implements Action {

    ShooterSubsystem shooterSubsystem;
    HardwareMap hardwareMap;
    boolean finished;

    public SetVeloPIDSAct(ShooterSubsystem shooterSubsystem, HardwareMap hardwareMap, boolean finished){
        this.shooterSubsystem = shooterSubsystem;
        this.hardwareMap = hardwareMap;
        this.finished = finished;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

        shooterSubsystem.setPIDs(
                Constants.P_GAIN,
                Constants.I_GAIN,
                Constants.D_GAIN
        );
        shooterSubsystem.setFeedForward(
                Constants.VEL_KS,
                Constants.VEL_KV * (12.0/hardwareMap.voltageSensor.iterator().next().getVoltage())
        );

        return !finished;
    }
}
