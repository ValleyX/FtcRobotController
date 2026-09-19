package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vcs.valleylib.core.command.Command;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.QualBotRobot;

/** Three shooting cycles from the near goal, red side. */
@Autonomous(name = "CB Red Near Goal", group = "QualBot Command")
public class RedCloseAuto extends AutoOpModeBase {

    private final CloseAutoGeometry geometry = CloseAutoGeometry.red();

    @Override
    protected Pose startingPose() {
        return geometry.start;
    }

    @Override
    protected int limelightPipeline() {
        return 1;
    }

    @Override
    protected Command buildRoutine(QualBotRobot robot) {
        return CloseAutoRoutine.build(robot, geometry);
    }
}
