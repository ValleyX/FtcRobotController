package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autos;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autos.AutoCommands.CommandAction;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autos.AutoCommands.IntakeSortAutoCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autos.AutoCommands.RunSmartSortShootAutoCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autos.AutoCommands.SmartSortShootAutoCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.StopIntakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.NeutralShooterCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.SlotCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.SavedVars;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Subsystems;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Roadrunner.MecanumDrive;

import java.util.List;


@Autonomous(name = "Blue Zamboni Far")
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
        initialPose = new Pose2d(-72 + (Constants.BOT_LENGTH/2.0),Constants.BOT_WIDTH/2.0, Math.toRadians(0.0));
        subsystems = new Subsystems(hardwareMap, Constants.BLUE_PIPELINE_MOTIF, initialPose);
        // instantiate MecanumDrive at a particular pose.



        pickup1 = subsystems.mecDriveSubsystem.drive.actionBuilder(new Pose2d(-72 + (Constants.BOT_LENGTH/2.0), Constants.BOT_WIDTH/2.0, Math.toRadians(0.0)))
                .setTangent(Math.toRadians(90.0))
                .splineToLinearHeading(new Pose2d(-36.0, 48.0, Math.toRadians(0.0)), Math.toRadians(0.0));


        moveToShoot2 = subsystems.mecDriveSubsystem.drive.actionBuilder(new Pose2d(-36.0, 48.0, Math.toRadians(0.0)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-60.0, 15.0, Math.toRadians(45.0)), Math.toRadians(90.0));


        pickup2 = subsystems.mecDriveSubsystem.drive.actionBuilder(new Pose2d(-60.0, 15.0, Math.toRadians(45.0)))
                .setReversed(false)
                .setTangent(Math.toRadians(90.0))
                .splineToLinearHeading(new Pose2d(-72 + (Constants.BOT_WIDTH/2.0), 72 - (Constants.BOT_LENGTH/2.0), Math.toRadians(90.0)), Math.toRadians(0.0));

        moveToShoot3 = subsystems.mecDriveSubsystem.drive.actionBuilder(new Pose2d(-72 + (Constants.BOT_WIDTH/2.0), 72 - (Constants.BOT_LENGTH/2.0), Math.toRadians(90.0)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-72 + (Constants.BOT_WIDTH/2.0), 20.0), Math.toRadians(90.0));

    }


    @Override
    public void runOpMode(){
        initialize();
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
            telemetry.addData("Pipeline: ", subsystems.sensorSubsystem.getPattern());
            telemetry.update();
        }
        waitForStart();
        if (isStopRequested()) return;
        subsystems.sensorSubsystem.setPipeline(Constants.BLUE_PIPELINE);

        Actions.runBlocking(new SequentialAction(
                new CommandAction(new SlotCmd(subsystems.spindexerSubsystem, subsystems.kickSubsystem, 0)),
                new CommandAction(new RunSmartSortShootAutoCmd(subsystems.shooterSubsystem, subsystems.shooterFeedSubsystem,
                        subsystems.sensorSubsystem, subsystems.aimSubsystem, subsystems.spindexerSubsystem,
                        subsystems.kickSubsystem, subsystems.intakeSubsystem, subsystems.mecDriveSubsystem, new Vector2d(subsystems.mecDriveSubsystem.getBotX(), subsystems.mecDriveSubsystem.getBotY()), subsystems.mecDriveSubsystem.getRobotHeading(), telemetry)),
                new CommandAction(new NeutralShooterCmd(subsystems.shooterSubsystem, subsystems.shooterFeedSubsystem,
                        subsystems.aimSubsystem, subsystems.kickSubsystem, subsystems.intakeSubsystem)),


                new ParallelAction(
                        new CommandAction(new IntakeSortAutoCmd(subsystems.intakeSubsystem, subsystems.spindexerSubsystem, subsystems.kickSubsystem)),
                        pickup1.build()
                ),

                moveToShoot2.build(),

                new CommandAction(new RunSmartSortShootAutoCmd(subsystems.shooterSubsystem, subsystems.shooterFeedSubsystem,
                        subsystems.sensorSubsystem, subsystems.aimSubsystem, subsystems.spindexerSubsystem,
                        subsystems.kickSubsystem, subsystems.intakeSubsystem, subsystems.mecDriveSubsystem, new Vector2d(subsystems.mecDriveSubsystem.getBotX(), subsystems.mecDriveSubsystem.getBotY()), subsystems.mecDriveSubsystem.getRobotHeading(), telemetry)),
                new CommandAction(new NeutralShooterCmd(subsystems.shooterSubsystem, subsystems.shooterFeedSubsystem,
                        subsystems.aimSubsystem, subsystems.kickSubsystem, subsystems.intakeSubsystem)),

                new ParallelAction(
                        new CommandAction(new IntakeSortAutoCmd(subsystems.intakeSubsystem, subsystems.spindexerSubsystem, subsystems.kickSubsystem)),
                        pickup2.build()
                ),

                moveToShoot3.build(),

                new CommandAction(new RunSmartSortShootAutoCmd(subsystems.shooterSubsystem, subsystems.shooterFeedSubsystem,
                        subsystems.sensorSubsystem, subsystems.aimSubsystem, subsystems.spindexerSubsystem,
                        subsystems.kickSubsystem, subsystems.intakeSubsystem, subsystems.mecDriveSubsystem, new Vector2d(subsystems.mecDriveSubsystem.getBotX(), subsystems.mecDriveSubsystem.getBotY()), subsystems.mecDriveSubsystem.getRobotHeading(), telemetry)),
                new CommandAction(new NeutralShooterCmd(subsystems.shooterSubsystem, subsystems.shooterFeedSubsystem,
                        subsystems.aimSubsystem, subsystems.kickSubsystem, subsystems.intakeSubsystem)),

                new ParallelAction(
                        new CommandAction(new IntakeSortAutoCmd(subsystems.intakeSubsystem, subsystems.spindexerSubsystem, subsystems.kickSubsystem)),
                        pickup2.build()
                ),

                moveToShoot3.build(),

                new CommandAction(new RunSmartSortShootAutoCmd(subsystems.shooterSubsystem, subsystems.shooterFeedSubsystem,
                        subsystems.sensorSubsystem, subsystems.aimSubsystem, subsystems.spindexerSubsystem,
                        subsystems.kickSubsystem, subsystems.intakeSubsystem, subsystems.mecDriveSubsystem, new Vector2d(subsystems.mecDriveSubsystem.getBotX(), subsystems.mecDriveSubsystem.getBotY()), subsystems.mecDriveSubsystem.getRobotHeading(), telemetry)),
                new CommandAction(new NeutralShooterCmd(subsystems.shooterSubsystem, subsystems.shooterFeedSubsystem,
                        subsystems.aimSubsystem, subsystems.kickSubsystem, subsystems.intakeSubsystem)),

                new ParallelAction(
                        new CommandAction(new IntakeSortAutoCmd(subsystems.intakeSubsystem, subsystems.spindexerSubsystem, subsystems.kickSubsystem)),
                        pickup2.build()
                ),

                moveToShoot3.build(),

                new CommandAction(new RunSmartSortShootAutoCmd(subsystems.shooterSubsystem, subsystems.shooterFeedSubsystem,
                        subsystems.sensorSubsystem, subsystems.aimSubsystem, subsystems.spindexerSubsystem,
                        subsystems.kickSubsystem, subsystems.intakeSubsystem, subsystems.mecDriveSubsystem, new Vector2d(subsystems.mecDriveSubsystem.getBotX(), subsystems.mecDriveSubsystem.getBotY()), subsystems.mecDriveSubsystem.getRobotHeading(), telemetry)),
                new CommandAction(new NeutralShooterCmd(subsystems.shooterSubsystem, subsystems.shooterFeedSubsystem,
                        subsystems.aimSubsystem, subsystems.kickSubsystem, subsystems.intakeSubsystem)),

                new ParallelAction(
                        new CommandAction(new IntakeSortAutoCmd(subsystems.intakeSubsystem, subsystems.spindexerSubsystem, subsystems.kickSubsystem)),
                        pickup2.build()
                ),

                moveToShoot3.build(),

                new CommandAction(new RunSmartSortShootAutoCmd(subsystems.shooterSubsystem, subsystems.shooterFeedSubsystem,
                        subsystems.sensorSubsystem, subsystems.aimSubsystem, subsystems.spindexerSubsystem,
                        subsystems.kickSubsystem, subsystems.intakeSubsystem, subsystems.mecDriveSubsystem, new Vector2d(subsystems.mecDriveSubsystem.getBotX(), subsystems.mecDriveSubsystem.getBotY()), subsystems.mecDriveSubsystem.getRobotHeading(), telemetry)),
                new CommandAction(new NeutralShooterCmd(subsystems.shooterSubsystem, subsystems.shooterFeedSubsystem,
                        subsystems.aimSubsystem, subsystems.kickSubsystem, subsystems.intakeSubsystem)),
                new ParallelAction(
                        new CommandAction(new IntakeSortAutoCmd(subsystems.intakeSubsystem, subsystems.spindexerSubsystem, subsystems.kickSubsystem)),
                        pickup2.build()
                )
        ));



        SavedVars.startingY = subsystems.mecDriveSubsystem.getBotY();
        SavedVars.startingX = subsystems.mecDriveSubsystem.getBotX();
        SavedVars.startingHeading = subsystems.mecDriveSubsystem.getRobotHeading();
        SavedVars.pattern = subsystems.sensorSubsystem.getPattern();
    }
}
