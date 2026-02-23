package com.vcs.valleylib.core.command;

public class InstantCommand implements Command {

    private final Runnable action;
    private boolean hasRun = false;

    public InstantCommand(Runnable action) {
        this.action = action;
    }

    @Override
    public void execute() {
        if (!hasRun) {
            action.run();
            hasRun = true;
        }
    }

    @Override
    public boolean isFinished() {
        return hasRun;
    }
}