package org.firstinspires.ftc.team12841.CompBot.autos.pedro;

import com.pedropathing.paths.Path;
import com.vcs.valleylib.core.command.Command;

import org.firstinspires.ftc.team12841.CompBot.RobotHardware;

public class FollowCmd implements Command {

    RobotHardware robot;
    Path path;
    public FollowCmd(RobotHardware robot, Path path){
        this.robot = robot;
        this.path = path;
    }
    @Override
    public void execute() {
        robot.updatePose();
        robot.getFollower().follow(path);
    }

    @Override
    public boolean isFinished() {
        return robot.getFollower().idle();
    }
}
