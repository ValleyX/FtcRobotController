package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.AutoCommands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class ActionDeadline implements Action {

    private final Action action;
    private final Action deadline;

    public ActionDeadline(Action action, Action deadline){
        this.action = action;
        this.deadline = deadline;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        action.run(telemetryPacket);
        return deadline.run(telemetryPacket);
    }
}
