package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.Autos;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.AutoCommands.SmartLineShooterAutoCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.IntakeLineCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.AutoCommands.CommandAction;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.AutoCommands.SavedVarsCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.AutoCommands.SmartSortShootAutoCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.StopIntakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.ResetCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.SavedVars;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Subsystems;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.Roadrunner.MecanumDrive;

import java.util.function.Supplier;

@Autonomous(name = "Blue Zamboni Close", group = "Autonomous")
public class BlueClose extends CommandOpMode {
    Subsystems subsystems;
    Pose2d initialPose;

//Trajectories and actions
    TrajectoryActionBuilder moveToShoot1;
    TrajectoryActionBuilder moveToShoot2;
    TrajectoryActionBuilder moveToShoot3;
    TrajectoryActionBuilder pickup1;
    TrajectoryActionBuilder pickup2;
    TrajectoryActionBuilder leave;

    //Command Actions
    CommandAction reset;

    Supplier<Pose2d> pose2dSupplier;

    @Override
    public void initialize() {
        initialPose = new Pose2d(55.0,45.0, Math.toRadians(45.0));
        subsystems = new Subsystems(hardwareMap, Constants.BLUE_PIPELINE_MOTIF, initialPose);
        // instantiate MecanumDrive at a particular pose.


        moveToShoot1 = subsystems.mecDriveSubsystem.drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(36.0, 36.0));


        pickup1 = subsystems.mecDriveSubsystem.drive.actionBuilder(new Pose2d(36.0, 36.0, Math.toRadians(45.0)))
                .setTangent(Math.toRadians(90.0))
                .splineToLinearHeading(new Pose2d(12.0, 48.0, Math.toRadians(180.0)), Math.toRadians(0.0));


        moveToShoot2 = subsystems.mecDriveSubsystem.drive.actionBuilder(new Pose2d(12.0, 48.0, Math.toRadians(180.0)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(36.0, 36.0, Math.toRadians(45.0)), Math.toRadians(180.0));


        pickup2 = subsystems.mecDriveSubsystem.drive.actionBuilder(new Pose2d(36.0, 36.0, Math.toRadians(45.0)))
                .setTangent(Math.toRadians(90.0))
                .splineToLinearHeading(new Pose2d(48.0, -12.0, Math.toRadians(180.0)), Math.toRadians(0.0));

        moveToShoot3 = subsystems.mecDriveSubsystem.drive.actionBuilder(new Pose2d(48.0, -12.0, Math.toRadians(180)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(36.0, -36.0, Math.toRadians(45)), Math.toRadians(180.0));

        leave = subsystems.mecDriveSubsystem.drive.actionBuilder(new Pose2d(36.0, -36.0, Math.toRadians(45.0)))
                .setTangent(Math.toRadians(135.0))
                .splineToLinearHeading(new Pose2d(0.0, 42.0, Math.toRadians(90.0)), Math.toRadians(0.0));

        reset = new CommandAction(new ResetCmd(subsystems.shooterSubsystem, subsystems.shooterFeedSubsystem,
                subsystems.spindexerSubsystem, subsystems.aimSubsystem, subsystems.kickSubsystem, subsystems.intakeSubsystem).withTimeout(10));

        pose2dSupplier = subsystems.mecDriveSubsystem.drive.localizer::getPose;
    }


    @Override
    public void runOpMode(){

        initialize();
        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        new CommandAction(new SavedVarsCmd(pose2dSupplier)),
                        new SequentialAction(
                                reset,
                                moveToShoot1.build(),

                                new CommandAction(new SmartLineShooterAutoCmd(subsystems.shooterSubsystem, subsystems.shooterFeedSubsystem,
                                    subsystems.sensorSubsystem, subsystems.aimSubsystem, subsystems.spindexerSubsystem,
                                    subsystems.kickSubsystem, subsystems.intakeSubsystem, subsystems.mecDriveSubsystem, telemetry).withTimeout(8000)),
                                reset,


                                new ParallelAction(
                                        new CommandAction(new IntakeLineCmd(subsystems.shooterFeedSubsystem, subsystems.intakeSubsystem, subsystems.spindexerSubsystem, subsystems.kickSubsystem).withTimeout(5000)),
                                        pickup1.build()
                                ),

                                moveToShoot2.build(),

                                new CommandAction(new StopIntakeCmd(subsystems.intakeSubsystem)),
                                new CommandAction(new SmartLineShooterAutoCmd(subsystems.shooterSubsystem, subsystems.shooterFeedSubsystem,
                                        subsystems.sensorSubsystem, subsystems.aimSubsystem, subsystems.spindexerSubsystem,
                                        subsystems.kickSubsystem, subsystems.intakeSubsystem, subsystems.mecDriveSubsystem, telemetry).withTimeout(8000)),
                                reset,

                                new ParallelAction(
                                        new CommandAction(new IntakeLineCmd(subsystems.shooterFeedSubsystem, subsystems.intakeSubsystem, subsystems.spindexerSubsystem, subsystems.kickSubsystem).withTimeout(5000)),
                                        pickup2.build()
                                ),

                                moveToShoot3.build(),

                                new CommandAction(new SmartLineShooterAutoCmd(subsystems.shooterSubsystem, subsystems.shooterFeedSubsystem,
                                        subsystems.sensorSubsystem, subsystems.aimSubsystem, subsystems.spindexerSubsystem,
                                        subsystems.kickSubsystem, subsystems.intakeSubsystem, subsystems.mecDriveSubsystem, telemetry).withTimeout(8000)),
                                reset,

                                leave.build()
                        )
                )
        );



        SavedVars.startingY = subsystems.mecDriveSubsystem.drive.localizer.getPose().position.y;
        SavedVars.startingX = subsystems.mecDriveSubsystem.drive.localizer.getPose().position.x;
        SavedVars.startingHeading = subsystems.mecDriveSubsystem.drive.localizer.getPose().heading.toDouble();
    }
}