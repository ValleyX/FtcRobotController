package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autos.AutoCommands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;

public class CommandAction implements Action {
    private final Command command;
    private boolean initialized = false;

    public CommandAction(Command command) {
        this.command = command;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (!initialized) {
            command.initialize();
            initialized = true;
        }

        command.execute();

        if (command.isFinished()) {
            command.end(false);
            return false; // Action is finished
        }

        return true; // Action is still running
    }
}
