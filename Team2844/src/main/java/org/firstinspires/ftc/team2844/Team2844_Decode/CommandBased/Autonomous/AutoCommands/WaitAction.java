package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.AutoCommands;

import android.telephony.TelephonyCallback;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class WaitAction implements Action {
    ElapsedTime time;
    double milliseconds;
    public WaitAction(double milliseconds){
        time = new ElapsedTime();
        time.reset();
        this.milliseconds = milliseconds;
    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        return time.time(TimeUnit.MILLISECONDS) < milliseconds;
    }
}
