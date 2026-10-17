package org.firstinspires.ftc.team2844.commands;

import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.command.decorators.SequentialCommandGroup;

public class CommandFactory {
    public static Command IntakeState(){
        return new SequentialCommandGroup(
                //start intake
                //start transfer and close gates
        );
    }

    public static Command ShootState(){
        return new SequentialCommandGroup(
                //
        );
    }
}
