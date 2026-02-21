package org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Autos.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Hardwares.QualConstants;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Hardwares.ShooterHardware;

public class IntakeAction implements Action {
    ShooterHardware shooterHardware;

    public IntakeAction(ShooterHardware shooterHardware){
        this.shooterHardware = shooterHardware;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        shooterHardware.closeServo();
        shooterHardware.stopBallHold();
        if(shooterHardware.threeBall()) {
            shooterHardware.intake(0.0);
        } else {
            shooterHardware.intake(QualConstants.INTAKE_SPEED);
        }

        return false;
    }
}
