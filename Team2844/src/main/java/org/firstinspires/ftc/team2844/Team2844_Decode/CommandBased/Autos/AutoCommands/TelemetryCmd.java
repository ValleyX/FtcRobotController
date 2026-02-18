package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autos.AutoCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetryCmd extends CommandBase {
    public TelemetryCmd(Telemetry telemetry, String caption, Object data){
        telemetry.addData(caption, data);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
