package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.LightCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ExtraSubsystems.LightSubsystem;

public class TimerLightsCmd extends SequentialCommandGroup {
    public TimerLightsCmd(LightSubsystem lightSubsystem, int index){
        addCommands(
                new SetLightTimedCmd(lightSubsystem, index, Constants.GREEN, 60000),
                new SetLightTimedCmd(lightSubsystem, index, Constants.YELLOW, 45000),
                new BlinkLightCmd(lightSubsystem, index, Constants.RED, Constants.BLACK, 1000, 15000)
        );
    }
}
