package com.vcs.valleylib.core.scheduler;

import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.subsystem.Subsystem;

import java.util.*;

public final class CommandScheduler {

    private static CommandScheduler instance;

    private final Set<Command> scheduledCommands = new HashSet<>();
    private final Set<Subsystem> subsystems = new HashSet<>();

    private CommandScheduler() {}

    public static CommandScheduler getInstance() {
        if (instance == null) {
            instance = new CommandScheduler();
        }
        return instance;
    }

    public void registerSubsystem(Subsystem subsystem) {
        subsystems.add(subsystem);
    }

    public void schedule(Command command) {
        command.initialize();
        scheduledCommands.add(command);
    }

    public void run() {
        for (Subsystem subsystem : subsystems) {
            subsystem.periodic();
        }

        Iterator<Command> iterator = scheduledCommands.iterator();
        while (iterator.hasNext()) {
            Command command = iterator.next();
            command.execute();

            if (command.isFinished()) {
                command.end(false);
                iterator.remove();
            }
        }
    }

    public void cancel(Command command) {
        if (scheduledCommands.remove(command)) {
            command.end(true);
        }
    }

    public void cancelAll() {
        for (Command command : scheduledCommands) {
            command.end(true);
        }
        scheduledCommands.clear();
    }
}