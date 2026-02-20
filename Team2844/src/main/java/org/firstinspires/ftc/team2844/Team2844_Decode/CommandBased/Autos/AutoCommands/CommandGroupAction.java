package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autos.AutoCommands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class CommandGroupAction implements Action {

    private final Command command;
    private boolean initialized = false;
    private boolean finished = false;
    private LinearOpMode opMode;

    public CommandGroupAction(Command command, LinearOpMode opMode) {
        this.command = command;
        this.opMode = opMode;
    }

    @Override
    public boolean run(TelemetryPacket telemetryPacket) {

        if (opMode.isStopRequested()) {
            CommandScheduler.getInstance().cancel(command);
            finished = true;
            return false;
        }

        if (!initialized) {
            CommandScheduler.getInstance().schedule(command);
            initialized = true;
        }

        if (!finished) {
            CommandScheduler.getInstance().run();

            if (!CommandScheduler.getInstance().isScheduled(command)) {
                finished = true;
            }
        }

        return !finished;
    }

}