package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.AutoCommands;

import com.arcrobotics.ftclib.command.CommandBase;

public class TimeoutCmd extends CommandBase {
    private final long timeoutMillis;
    private long startTime;

    public TimeoutCmd(long timeoutMillis) {
        this.timeoutMillis = timeoutMillis;
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime >= timeoutMillis;
    }
}
