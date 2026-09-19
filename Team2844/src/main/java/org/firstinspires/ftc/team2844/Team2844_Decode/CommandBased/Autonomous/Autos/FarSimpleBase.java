package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.Autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.AutoCommands.ActionDeadline;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.AutoCommands.CommandAction;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.AutoCommands.IntakeLineAct;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.AutoCommands.ResetAction;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.AutoCommands.SetVeloPIDSAct;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.AutoCommands.SmartLineShooterAutoAct;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.StopIntakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.VelocityShootCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.SavedVars;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Subsystems;

@Disabled
public class FarSimpleBase extends LinearOpMode {

    Subsystems subsystems;
    Pose2d initialPose;
    TrajectoryActionBuilder moveToShoot1;
    TrajectoryActionBuilder moveToShoot2;
    TrajectoryActionBuilder pickup1;
    TrajectoryActionBuilder leave;

    Action reset;

    public int pipeline;
    int flip;
    public boolean red;

    public void initialize(){
        if(pipeline == Constants.RED_PIPELINE || pipeline == Constants.RED_PIPELINE_MOTIF){
            red = true;
            flip = -1;
        } else {
            red = false;
            flip = 1;
        }

        initialPose = new Pose2d(72.0 - Constants.BOT_WIDTH/2.0,(Constants.BOT_LENGTH/2.0)*flip, Math.toRadians(-90.0*flip));
        subsystems = new Subsystems(hardwareMap, pipeline, initialPose);
        // instantiate MecanumDrive at a particular pose.


        pickup1 = subsystems.mecDriveSubsystem.drive.actionBuilder(initialPose)
                .lineToY(Constants.FPICKUP1_Y * flip);

    //.setTangent(Math.toRadians(-90.0*flip))

    //.splineToConstantHeading(new Vector2d(Constants.FPICKUP1_X, Constants.FPICKUP1_Y *flip), Math.toRadians(90.0*flip));


        moveToShoot2 = subsystems.mecDriveSubsystem.drive.actionBuilder(new Pose2d(Constants.FPICKUP1_X, Constants.FPICKUP1_Y *flip, Math.toRadians(Constants.FPICKUP1_DEGREES *flip)))
                .lineToY(Constants.FSHOOT_SPOT_Y* flip);
//                .setTangent(Math.toRadians(90.0*flip))
//                .splineToConstantHeading(new Vector2d(Constants.FSHOOT_SPOT_X, Constants.FSHOOT_SPOT_Y *flip, Math.toRadians(Constants.FSHOOT_DEGREES *flip)), Math.toRadians(-90.0*flip));

        leave = subsystems.mecDriveSubsystem.drive.actionBuilder(new Pose2d(Constants.FSHOOT_SPOT_X, Constants.FSHOOT_SPOT_Y *flip, Math.toRadians(Constants.FSHOOT_DEGREES *flip)))
                .lineToY(Constants.FENDY* flip);

        reset = (new ResetAction(subsystems.shooterFeedSubsystem, subsystems.kickSubsystem, subsystems.intakeSubsystem));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        if (isStopRequested()) return;

        try{
            Actions.runBlocking(
                    new SequentialAction(
                            new SetVeloPIDSAct(subsystems.shooterSubsystem, hardwareMap, true),
                            new CommandAction(new VelocityShootCmd(subsystems.shooterSubsystem, () -> 1300)),
                            new ActionDeadline(
                                    new SetVeloPIDSAct(subsystems.shooterSubsystem, hardwareMap, false),
                                    new SequentialAction(

                                            new SmartLineShooterAutoAct(subsystems.shooterSubsystem, subsystems.shooterFeedSubsystem,
                                                    subsystems.sensorSubsystem, subsystems.aimSubsystem, subsystems.spindexerSubsystem,
                                                    subsystems.kickSubsystem, subsystems.intakeSubsystem, subsystems.mecDriveSubsystem, telemetry),
                                            reset,


                                            new ActionDeadline(
                                                    new IntakeLineAct(subsystems.shooterFeedSubsystem, subsystems.intakeSubsystem, subsystems.spindexerSubsystem, subsystems.kickSubsystem),
                                                    new SequentialAction(
                                                            pickup1.build(),
                                                            new ParallelAction(
                                                                    new CommandAction(new VelocityShootCmd(subsystems.shooterSubsystem, () -> 1300)),
                                                                    moveToShoot2.build()
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
                            leave.build()
                    )
            );
        }finally {
            Pose2d pose = subsystems.mecDriveSubsystem.drive.localizer.getPose();
            SavedVars.startingY = pose.position.y;
            SavedVars.startingX = pose.position.x;
            SavedVars.startingHeading = pose.heading.toDouble();
        }
    }
}
