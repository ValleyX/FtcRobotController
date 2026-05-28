package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.Autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.AutoCommands.ActionDeadline;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.AutoCommands.IntakeLineAct;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.AutoCommands.ResetAction;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.AutoCommands.SetVeloPIDSAct;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.AutoCommands.SmartLineShooterAutoAct;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.AutoCommands.TimeoutCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.AimTurretCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.FullAimToLLCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.IntakeLineCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.AutoCommands.CommandAction;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.AutoCommands.SavedVarsCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.StopIntakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.SetVeloPIDS;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.VelocityShootCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.SavedVars;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Subsystems;

import java.util.function.Supplier;

//@Autonomous(name = "Blue Zamboni Close", group = "Autonomous")
@Disabled
public class CloseAutoBase extends LinearOpMode {
    Subsystems subsystems;
    Pose2d initialPose;
    OpMode opMode_;

    //Trajectories and actions
    TrajectoryActionBuilder moveToShoot1;
    TrajectoryActionBuilder moveToShoot2;
    TrajectoryActionBuilder moveToShoot3;
    TrajectoryActionBuilder pickup1;
    TrajectoryActionBuilder pickup2;
    TrajectoryActionBuilder leave;

    //Command Actions
    Action reset;

    Supplier<Pose2d> pose2dSupplier;

    public int pipeline;
    int flip;
    public boolean red;

    public void initialize() {
        if(pipeline == Constants.RED_PIPELINE || pipeline == Constants.RED_PIPELINE_MOTIF){
            red = true;
            flip = -1;
        } else {
            red = false;
            flip = 1;
        }

        initialPose = new Pose2d(-58.0,-45.0*flip, Math.toRadians(-135.0*flip));
        subsystems = new Subsystems(hardwareMap, pipeline, initialPose);
        // instantiate MecanumDrive at a particular pose.


        moveToShoot1 = subsystems.mecDriveSubsystem.drive.actionBuilder(initialPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(Constants.CSHOOT_SPOT_X, Constants.CSHOOT_SPOT_Y *flip, Math.toRadians(Constants.CSHOOT_DEGREES*flip)), Math.toRadians(45.0*flip));


        pickup1 = subsystems.mecDriveSubsystem.drive.actionBuilder(new Pose2d(Constants.CSHOOT_SPOT_X, Constants.CSHOOT_SPOT_Y *flip, Math.toRadians(Constants.CSHOOT_DEGREES *flip)))
                .turnTo( Math.toRadians(Constants.CPICKUP1_DEGREES *flip))
                .setTangent(Math.toRadians(-90.0*flip))
                .splineToConstantHeading(new Vector2d(Constants.CPICKUP1_X, Constants.CPICKUP1_Y *flip), Math.toRadians(-10.0*flip));


        moveToShoot2 = subsystems.mecDriveSubsystem.drive.actionBuilder(new Pose2d(Constants.CPICKUP1_X, Constants.CPICKUP1_Y *flip, Math.toRadians(Constants.CPICKUP1_DEGREES *flip)))
                .turnTo( Math.toRadians((Constants.CSHOOT_DEGREES-1)*flip))
                .setTangent(Math.toRadians(90.0*flip))
                .splineToLinearHeading(new Pose2d(Constants.CSHOOT_SPOT_X, Constants.CSHOOT_SPOT_Y *flip, Math.toRadians(Constants.CSHOOT_DEGREES *flip)), Math.toRadians(90.0*flip));


        pickup2 = subsystems.mecDriveSubsystem.drive.actionBuilder(new Pose2d(Constants.CSHOOT_SPOT_X, Constants.CSHOOT_SPOT_Y *flip, Math.toRadians(Constants.CSHOOT_DEGREES *flip)))
                .turnTo( Math.toRadians(Constants.CPICKUP2_DEGREES *flip))
                .setTangent(Math.toRadians(-90.0*flip))
                .splineToConstantHeading(new Vector2d(Constants.CPICKUP2_X, Constants.CPICKUP2_Y *flip), Math.toRadians(0.0*flip));

        moveToShoot3 = subsystems.mecDriveSubsystem.drive.actionBuilder(new Pose2d(Constants.CPICKUP2_X, Constants.CPICKUP2_Y *flip, Math.toRadians(Constants.CPICKUP2_DEGREES *flip)))
                .setReversed(!red)
                .splineToSplineHeading(new Pose2d(Constants.CSHOOT_SPOT_X, Constants.CSHOOT_SPOT_Y *flip, Math.toRadians(Constants.CSHOOT_DEGREES *flip)), Math.toRadians(90.0*flip));

        leave = subsystems.mecDriveSubsystem.drive.actionBuilder(new Pose2d(Constants.CSHOOT_SPOT_X, Constants.CSHOOT_SPOT_Y *flip, Math.toRadians(Constants.CSHOOT_DEGREES *flip)))
                .setTangent(Math.toRadians(180.0*flip))
                .splineToLinearHeading(new Pose2d(Constants.CEND_X, Constants.CEND_Y *flip, Math.toRadians(Constants.CEND_DEGREES *flip)), Math.toRadians(90.0*flip));

        reset = (new ResetAction(subsystems.shooterFeedSubsystem, subsystems.kickSubsystem, subsystems.intakeSubsystem));

        pose2dSupplier = subsystems.mecDriveSubsystem.drive.localizer::getPose;
    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();
        if (isStopRequested()) return;

        try {
            Actions.runBlocking(
                    new SequentialAction(
                            new CommandAction(new VelocityShootCmd(subsystems.shooterSubsystem, () -> 13000)),
                            new ActionDeadline(
                                    new SetVeloPIDSAct(subsystems.shooterSubsystem, hardwareMap, false),
                                    new SequentialAction(
                                            new ParallelAction(
                                                    new CommandAction(new VelocityShootCmd(subsystems.shooterSubsystem, () -> 1000)),
                                                    moveToShoot1.build()
                                            ),

                                            new SmartLineShooterAutoAct(subsystems.shooterSubsystem, subsystems.shooterFeedSubsystem,
                                                    subsystems.sensorSubsystem, subsystems.aimSubsystem, subsystems.spindexerSubsystem,
                                                    subsystems.kickSubsystem, subsystems.intakeSubsystem, subsystems.mecDriveSubsystem, telemetry),
                                            reset,


                                            new ActionDeadline(
                                                    new IntakeLineAct(subsystems.shooterFeedSubsystem, subsystems.intakeSubsystem, subsystems.spindexerSubsystem, subsystems.kickSubsystem),
                                                    new SequentialAction(
                                                        pickup1.build(),
                                                            new ParallelAction(
                                                                    new CommandAction(new VelocityShootCmd(subsystems.shooterSubsystem, () -> 940)),
                                                                    moveToShoot2.build()
                                                            )
                                                    )
                                            ),
                                            new CommandAction(new StopIntakeCmd(subsystems.intakeSubsystem)),


                                            new SmartLineShooterAutoAct(subsystems.shooterSubsystem, subsystems.shooterFeedSubsystem,
                                                    subsystems.sensorSubsystem, subsystems.aimSubsystem, subsystems.spindexerSubsystem,
                                                    subsystems.kickSubsystem, subsystems.intakeSubsystem, subsystems.mecDriveSubsystem, telemetry),
                                            reset,

                                            new ActionDeadline(
                                                    new IntakeLineAct(subsystems.shooterFeedSubsystem, subsystems.intakeSubsystem, subsystems.spindexerSubsystem, subsystems.kickSubsystem),
                                                    new SequentialAction(
                                                            pickup2.build(),
                                                            new ParallelAction(
                                                                    new CommandAction(new VelocityShootCmd(subsystems.shooterSubsystem, () -> 940)),
                                                                    moveToShoot3.build()
                                                            )
                                                    )
                                            ),
                                            new CommandAction(new StopIntakeCmd(subsystems.intakeSubsystem)),

                                            new SmartLineShooterAutoAct(subsystems.shooterSubsystem, subsystems.shooterFeedSubsystem,
                                                    subsystems.sensorSubsystem, subsystems.aimSubsystem, subsystems.spindexerSubsystem,
                                                    subsystems.kickSubsystem, subsystems.intakeSubsystem, subsystems.mecDriveSubsystem, telemetry),
                                            reset
                                    )
                            ),
                            new ParallelAction(
                                new CommandAction(new AimTurretCmd(subsystems.aimSubsystem, 180.0)),
                                leave.build()
                            )
                    )
            );
        } finally {
            Pose2d pose = subsystems.mecDriveSubsystem.drive.localizer.getPose();
            SavedVars.startingY = pose.position.y;
            SavedVars.startingX = pose.position.x;
            SavedVars.startingHeading = pose.heading.toDouble();
        }
    }
}