package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.DriveCommands;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.DriveSubsystem;

public class ResetPoseCmd extends CommandBase {

    DriveSubsystem driveSubsystem;
    int pipeline;

    public ResetPoseCmd(DriveSubsystem driveSubsystem, int pipeline){
        this.driveSubsystem = driveSubsystem;
        this.pipeline = pipeline;
    }

    @Override
    public void initialize() {
        if(pipeline == Constants.BLUE_PIPELINE || pipeline == Constants.BLUE_PIPELINE_MOTIF) {
            driveSubsystem.setPinpointPose(new Pose2d(-72.0 + Constants.BOT_WIDTH/2.0, 72.0 + Constants.BOT_WIDTH/2.0, 0.0));
        } else {
            driveSubsystem.setPinpointPose(new Pose2d(-72.0 + Constants.BOT_WIDTH/2.0, -72.0 + Constants.BOT_WIDTH/2.0, 0.0));
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
