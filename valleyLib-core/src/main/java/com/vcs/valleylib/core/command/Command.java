package com.vcs.valleylib.core.command;

public interface Command {

    default void initialize() {}

    void execute();

    default void end(boolean interrupted) {}

    default boolean isFinished() {
        return false;
    }
}