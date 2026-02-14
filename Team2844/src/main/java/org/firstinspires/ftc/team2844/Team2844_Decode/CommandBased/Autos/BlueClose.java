package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autos.AutoSubsystems.LimelightSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.SavedVars;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Subsystems;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Roadrunner.PinpointLocalizer;

import java.util.Collections;
import java.util.Set;

public class BlueClose extends CommandOpMode {
    Subsystems subsystems;
    LimelightSubsystem limelightSubsystem;
    Pose2d initialPose;
    MecanumDrive drive;

//Trajectories and actions
    TrajectoryActionBuilder moveToShoot1;

    @Override
    public void initialize() {
        subsystems = new Subsystems(hardwareMap, 0, true);
        limelightSubsystem = new LimelightSubsystem(hardwareMap, 3);
        // instantiate MecanumDrive at a particular pose.
        initialPose = new Pose2d(55,-45, Math.toRadians(45));
        drive = new MecanumDrive(hardwareMap, initialPose);


        moveToShoot1 = drive.actionBuilder(initialPose)
                .lineToYConstantHeading(-36);

    }


    @Override
    public void runOpMode(){

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(
            moveToShoot1.build()

        ));



        SavedVars.startingY = drive.getRobotY();
        SavedVars.startingX = drive.getRobotX();
        SavedVars.startingHeading = drive.getRobotHeading();
    }
}