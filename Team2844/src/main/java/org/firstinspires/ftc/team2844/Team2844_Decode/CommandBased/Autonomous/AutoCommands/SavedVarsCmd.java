package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.AutoCommands;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.SavedVars;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.DriveSubsystem;

import java.util.function.Supplier;

public class SavedVarsCmd extends CommandBase {

    DriveSubsystem mecDrive;
    public SavedVarsCmd(DriveSubsystem mecDrive){
        this.mecDrive = mecDrive;
    }

    @Override
    public void execute() {
        //mecDrive.drive.localizer.update();
        Pose2d pose = mecDrive.drive.localizer.getPose();
        SavedVars.startingY = pose.position.y;
        SavedVars.startingX = pose.position.x;
        SavedVars.startingHeading = pose.heading.toDouble();

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
