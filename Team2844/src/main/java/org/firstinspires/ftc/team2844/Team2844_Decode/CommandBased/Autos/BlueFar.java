package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autos;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autos.AutoCommands.CommandAction;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autos.AutoCommands.IntakeSortAutoCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autos.AutoCommands.SmartSortShootAutoCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.StopIntakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.NeutralShooterCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.SavedVars;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Subsystems;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Roadrunner.MecanumDrive;

import java.util.List;

public class BlueFar extends CommandOpMode {
    Subsystems subsystems;
    Pose2d initialPose;
    MecanumDrive drive;

    //Trajectories and actions
    TrajectoryActionBuilder moveToShoot2;
    TrajectoryActionBuilder moveToShoot3;
    TrajectoryActionBuilder pickup1;
    TrajectoryActionBuilder pickup2;

    @Override
    public void initialize() {
        subsystems = new Subsystems(hardwareMap, Constants.BLUE_PIPELINE_MOTIF, true);
        // instantiate MecanumDrive at a particular pose.
        initialPose = new Pose2d(-72 + (Constants.BOT_LENGTH/2.0),Constants.BOT_WIDTH/2.0, Math.toRadians(0.0));
        drive = new MecanumDrive(hardwareMap, initialPose);



        pickup1 = drive.actionBuilder(new Pose2d(-72 + (Constants.BOT_LENGTH/2.0), Constants.BOT_WIDTH/2.0, Math.toRadians(0.0)))
                .setTangent(Math.toRadians(90.0))
                .splineToLinearHeading(new Pose2d(-36.0, 48.0, Math.toRadians(0.0)), Math.toRadians(0.0));


        moveToShoot2 = drive.actionBuilder(new Pose2d(-36.0, 48.0, Math.toRadians(0.0)))
                .setReversed(true)
                .setTangent(Math.toRadians(90.0))
                .splineToLinearHeading(new Pose2d(-60.0, 15.0, Math.toRadians(45.0)), Math.toRadians(90.0));


        pickup2 = drive.actionBuilder(new Pose2d(-60.0, 15.0, Math.toRadians(45.0)))
                .setReversed(false)
                .setTangent(Math.toRadians(90.0))
                .splineToLinearHeading(new Pose2d(-72 + (Constants.BOT_WIDTH/2.0), 72 - (Constants.BOT_LENGTH/2.0), Math.toRadians(90.0)), Math.toRadians(0.0));

        moveToShoot3 = drive.actionBuilder(new Pose2d(-72 + (Constants.BOT_WIDTH/2.0), 72 - (Constants.BOT_LENGTH/2.0), Math.toRadians(90.0)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-72 + (Constants.BOT_WIDTH/2.0), 20.0), Math.toRadians(90.0));


        while(!isStopRequested() && opModeInInit()){
            List<LLResultTypes.FiducialResult> fiducials = subsystems.sensorSubsystem.getLatestResult().getFiducialResults();
            for (int i = 0; i < fiducials.size(); i++){
                if(fiducials.get(i).getFiducialId() == 21){
                    subsystems.sensorSubsystem.setPattern(Constants.PATTERN_GPP);
                } else if (fiducials.get(i).getFiducialId() == 22){
                    subsystems.sensorSubsystem.setPattern(Constants.PATTERN_PGP);
                } else if (fiducials.get(i).getFiducialId() == 23){
                    subsystems.sensorSubsystem.setPattern(Constants.PATTERN_PPG);
                }
            }
        }

    }


    @Override
    public void runOpMode(){

        waitForStart();
        if (isStopRequested()) return;
        subsystems.sensorSubsystem.setPipeline(Constants.BLUE_PIPELINE);

        Actions.runBlocking(new SequentialAction(
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
                new ParallelAction(
                        new CommandAction(new IntakeSortAutoCmd(subsystems.intakeSubsystem, subsystems.spindexerSubsystem, subsystems.kickSubsystem)),
                        pickup2.build()
                )
        ));



        SavedVars.startingY = drive.getRobotY();
        SavedVars.startingX = drive.getRobotX();
        SavedVars.startingHeading = drive.getRobotHeading();
        SavedVars.pattern = subsystems.sensorSubsystem.getPattern();
    }
}
