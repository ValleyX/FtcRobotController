package org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Autos.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Hardwares.QualConstants;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Hardwares.ShooterHardware;

public class StopIntakeAction implements Action {
    ShooterHardware shooterHardware;

    public StopIntakeAction(ShooterHardware shooterHardware){
        this.shooterHardware = shooterHardware;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        shooterHardware.intake(0.0);
        shooterHardware.closeServo();
        return false;
    }
}
