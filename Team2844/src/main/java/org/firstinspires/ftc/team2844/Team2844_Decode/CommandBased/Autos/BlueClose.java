package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autos;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autos.AutoCommands.CommandAction;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autos.AutoCommands.IntakeSortAutoCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autos.AutoCommands.SmartSortShootAutoCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.StopIntakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.NeutralShooterCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.SavedVars;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Subsystems;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Roadrunner.MecanumDrive;

public class BlueClose extends CommandOpMode {
    Subsystems subsystems;
    Pose2d initialPose;
    MecanumDrive drive;

//Trajectories and actions
    TrajectoryActionBuilder moveToShoot1;
    TrajectoryActionBuilder moveToShoot2;
    TrajectoryActionBuilder moveToShoot3;
    TrajectoryActionBuilder pickup1;
    TrajectoryActionBuilder pickup2;
    TrajectoryActionBuilder leave;

    @Override
    public void initialize() {
        subsystems = new Subsystems(hardwareMap, Constants.BLUE_PIPELINE_MOTIF, true);
        // instantiate MecanumDrive at a particular pose.
        initialPose = new Pose2d(55.0,45.0, Math.toRadians(45.0));
        drive = new MecanumDrive(hardwareMap, initialPose);


        moveToShoot1 = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(36.0, 36.0));


        pickup1 = drive.actionBuilder(new Pose2d(36.0, 36.0, Math.toRadians(45.0)))
                .setTangent(Math.toRadians(90.0))
                .splineToLinearHeading(new Pose2d(12.0, 48.0, Math.toRadians(180.0)), Math.toRadians(0.0));


        moveToShoot2 = drive.actionBuilder(new Pose2d(12.0, 48.0, Math.toRadians(180.0)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(36.0, 36.0, Math.toRadians(45.0)), Math.toRadians(180.0));


        pickup2 = drive.actionBuilder(new Pose2d(36.0, 36.0, Math.toRadians(45.0)))
                .setTangent(Math.toRadians(90.0))
                .splineToLinearHeading(new Pose2d(48.0, -12.0, Math.toRadians(180.0)), Math.toRadians(0.0));

        moveToShoot3 = drive.actionBuilder(new Pose2d(48.0, -12.0, Math.toRadians(180)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(36.0, -36.0, Math.toRadians(45)), Math.toRadians(180.0));

        leave = drive.actionBuilder(new Pose2d(36.0, -36.0, Math.toRadians(45.0)))
                .setTangent(Math.toRadians(135.0))
                .splineToLinearHeading(new Pose2d(0.0, 42.0, Math.toRadians(90.0)), Math.toRadians(0.0));

    }


    @Override
    public void runOpMode(){

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(
                new CommandAction(new NeutralShooterCmd(subsystems.shooterSubsystem, subsystems.shooterFeedSubsystem,
                        subsystems.aimSubsystem, subsystems.kickSubsystem, subsystems.intakeSubsystem)),

                moveToShoot1.build(),

                new CommandAction(new SmartSortShootAutoCmd(subsystems.shooterSubsystem, subsystems.shooterFeedSubsystem,
                    subsystems.sensorSubsystem, subsystems.aimSubsystem, subsystems.spindexerSubsystem,
                    subsystems.kickSubsystem, subsystems.intakeSubsystem, new Vector2d(drive.getRobotX(), drive.getRobotY()), drive.getRobotHeading())),
                new CommandAction(new NeutralShooterCmd(subsystems.shooterSubsystem, subsystems.shooterFeedSubsystem,
                        subsystems.aimSubsystem, subsystems.kickSubsystem, subsystems.intakeSubsystem)),


                new ParallelAction(
                        new CommandAction(new IntakeSortAutoCmd(subsystems.intakeSubsystem, subsystems.spindexerSubsystem, subsystems.kickSubsystem)),
                        pickup1.build()
                ),

                moveToShoot2.build(),

                new CommandAction(new StopIntakeCmd(subsystems.intakeSubsystem)),
                new CommandAction(new SmartSortShootAutoCmd(subsystems.shooterSubsystem, subsystems.shooterFeedSubsystem,
                        subsystems.sensorSubsystem, subsystems.aimSubsystem, subsystems.spindexerSubsystem,
                        subsystems.kickSubsystem, subsystems.intakeSubsystem, new Vector2d(drive.getRobotX(), drive.getRobotY()), drive.getRobotHeading())),
                new CommandAction(new NeutralShooterCmd(subsystems.shooterSubsystem, subsystems.shooterFeedSubsystem,
                        subsystems.aimSubsystem, subsystems.kickSubsystem, subsystems.intakeSubsystem)),

                new ParallelAction(
                        new CommandAction(new IntakeSortAutoCmd(subsystems.intakeSubsystem, subsystems.spindexerSubsystem, subsystems.kickSubsystem)),
                        pickup2.build()
                ),

                moveToShoot3.build(),

                new CommandAction(new SmartSortShootAutoCmd(subsystems.shooterSubsystem, subsystems.shooterFeedSubsystem,
                        subsystems.sensorSubsystem, subsystems.aimSubsystem, subsystems.spindexerSubsystem,
                        subsystems.kickSubsystem, subsystems.intakeSubsystem, new Vector2d(drive.getRobotX(), drive.getRobotY()), drive.getRobotHeading())),
                new CommandAction(new NeutralShooterCmd(subsystems.shooterSubsystem, subsystems.shooterFeedSubsystem,
                        subsystems.aimSubsystem, subsystems.kickSubsystem, subsystems.intakeSubsystem)),

                leave.build()
        ));



        SavedVars.startingY = drive.getRobotY();
        SavedVars.startingX = drive.getRobotX();
        SavedVars.startingHeading = drive.getRobotHeading();
    }
}