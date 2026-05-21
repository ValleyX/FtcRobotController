package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.DriveCommands;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.DriveSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.SensorSubsystem;

public class ResetPoseCmd extends CommandBase {

    DriveSubsystem driveSubsystem;
    SensorSubsystem sensorSubsystem;
    int pipeline;

    public ResetPoseCmd(DriveSubsystem driveSubsystem, SensorSubsystem sensorSubsystem, int pipeline){
        this.driveSubsystem = driveSubsystem;
        this.sensorSubsystem = sensorSubsystem;
        this.pipeline = pipeline;
    }

    @Override
    public void initialize() {
        if(sensorSubsystem.getTx() != Constants.NO_LL){
            driveSubsystem.setPinpointPose(new Pose2d(sensorSubsystem.getBotXLLMT2(), sensorSubsystem.getBotYLLMT2(), driveSubsystem.getRobotHeading()));
        } else if(pipeline == Constants.BLUE_PIPELINE || pipeline == Constants.BLUE_PIPELINE_MOTIF) {
            driveSubsystem.setPinpointPose(new Pose2d(72.0 - Constants.BOT_WIDTH/2.0, 72.0 - Constants.BOT_WIDTH/2.0, Math.toRadians(-90.0)));
        } else {
            driveSubsystem.setPinpointPose(new Pose2d(72.0 - Constants.BOT_WIDTH/2.0, -72.0 + Constants.BOT_WIDTH/2.0, Math.toRadians(90.0)));
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
