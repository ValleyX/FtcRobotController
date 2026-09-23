package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vcs.valleylib.core.command.Command;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.QualBotRobot;

/** Shoot the preload from the far goal and clear the start area. Blue side. */
@Autonomous(name = "CB Blue Far Goal 3 Ball", group = "QualBot Command")
public class BlueFarAuto extends AutoOpModeBase {

    @Override
    protected Pose startingPose() {
        return FarAutoRoutines.BLUE_START;
    }

    @Override
    protected int limelightPipeline() {
        return 0;
    }

    @Override
    protected Command buildRoutine(QualBotRobot robot) {
        return FarAutoRoutines.blueThreeBall(robot);
    }
}
