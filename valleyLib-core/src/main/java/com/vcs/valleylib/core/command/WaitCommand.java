package com.vcs.valleylib.core.command;

public class WaitCommand implements Command {

    private final long waitTimeMs;
    private long startTime;

    public WaitCommand(double seconds) {
        this.waitTimeMs = (long) (seconds * 1000);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime >= waitTimeMs;
    }
}