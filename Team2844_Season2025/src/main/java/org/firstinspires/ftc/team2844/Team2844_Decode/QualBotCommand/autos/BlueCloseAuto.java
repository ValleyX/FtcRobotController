package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vcs.valleylib.core.command.Command;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.QualBotRobot;

/** Three shooting cycles from the near goal, blue side. */
@Autonomous(name = "CB Blue Near Goal", group = "QualBot Command")
public class BlueCloseAuto extends AutoOpModeBase {

    private final CloseAutoGeometry geometry = CloseAutoGeometry.blue();

    @Override
    protected Pose startingPose() {
        return geometry.start;
    }

    @Override
    protected int limelightPipeline() {
        return 0;
    }

    @Override
    protected Command buildRoutine(QualBotRobot robot) {
        return CloseAutoRoutine.build(robot, geometry);
    }
}
