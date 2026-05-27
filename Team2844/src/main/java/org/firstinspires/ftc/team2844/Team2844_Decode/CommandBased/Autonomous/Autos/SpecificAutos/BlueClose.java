package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.Autos.SpecificAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.Autos.CloseAutoBase;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;

@Autonomous(name = "Blue Zamboni Close", group = "Autonomous")
public class BlueClose extends CloseAutoBase {
    @Override
    public void initialize() {
        super.pipeline = Constants.BLUE_PIPELINE;
        super.initialize();
    }
}
    /*Subsystems subsystems;
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
    RaceAction reset;

    Supplier<Pose2d> pose2dSupplier;

    @Override
    public void initialize() {
        initialPose = new Pose2d(-55.0,-45.0, Math.toRadians(-135.0));
        subsystems = new Subsystems(hardwareMap, Constants.BLUE_PIPELINE, initialPose);
        // instantiate MecanumDrive at a particular pose.


        moveToShoot1 = subsystems.mecDriveSubsystem.drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(Constants.CSHOOT_SPOT_X, Constants.CSHOOT_SPOT_Y));


        pickup1 = subsystems.mecDriveSubsystem.drive.actionBuilder(new Pose2d(Constants.CSHOOT_SPOT_X, Constants.CSHOOT_SPOT_Y, Math.toRadians(Constants.CSHOOT_DEGREES)))
                .setTangent(Math.toRadians(-45.0))
                .splineToSplineHeading(new Pose2d(Constants.CPICKUP1_X, Constants.CPICKUP1_Y, Math.toRadians(Constants.CPICKUP1_DEGREES)), Math.toRadians(0.0));


        moveToShoot2 = subsystems.mecDriveSubsystem.drive.actionBuilder(new Pose2d(Constants.CPICKUP1_X, Constants.CPICKUP1_Y, Math.toRadians(Constants.CPICKUP1_DEGREES)))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(Constants.CSHOOT_SPOT_X, Constants.CSHOOT_SPOT_Y, Math.toRadians(Constants.CSHOOT_DEGREES)), Math.toRadians(0.0));


        pickup2 = subsystems.mecDriveSubsystem.drive.actionBuilder(new Pose2d(Constants.CSHOOT_SPOT_X, Constants.CSHOOT_SPOT_Y, Math.toRadians(Constants.CSHOOT_DEGREES)))
                .setTangent(Math.toRadians(-45.0))
                .splineToSplineHeading(new Pose2d(Constants.CPICKUP2_X, Constants.CPICKUP2_Y, Math.toRadians(Constants.CPICKUP2_DEGREES)), Math.toRadians(0.0));

        moveToShoot3 = subsystems.mecDriveSubsystem.drive.actionBuilder(new Pose2d(Constants.CPICKUP2_X, Constants.CPICKUP2_Y, Math.toRadians(Constants.CPICKUP2_DEGREES)))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(Constants.CSHOOT_SPOT_X, Constants.CSHOOT_SPOT_Y, Math.toRadians(Constants.CSHOOT_DEGREES)), Math.toRadians(180.0));

        leave = subsystems.mecDriveSubsystem.drive.actionBuilder(new Pose2d(Constants.CSHOOT_SPOT_X, Constants.CSHOOT_SPOT_Y, Math.toRadians(Constants.CSHOOT_DEGREES)))
                .setTangent(Math.toRadians(0.0))
                .splineToSplineHeading(new Pose2d(Constants.CEND_X, Constants.CEND_Y, Math.toRadians(Constants.CEND_DEGREES)), Math.toRadians(90.0));

        reset = new RaceAction((new ResetAction(subsystems.shooterFeedSubsystem, subsystems.kickSubsystem, subsystems.intakeSubsystem)),
                new CommandAction(new TimeoutCmd(10)));

        pose2dSupplier = subsystems.mecDriveSubsystem.drive.localizer::getPose;
    }


    @Override
    public void runOpMode(){

        initialize();
        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new RaceAction(
                        new SequentialAction(
                                new CommandAction(new VelocityShootCmd(subsystems.shooterSubsystem, () -> 1000)),
                                new ParallelAction(
                                        //new CommandAction(new SavedVarsCmd(pose2dSupplier)),
                                        new CommandAction(new SetVeloPIDS(subsystems.shooterSubsystem, hardwareMap)),
                                        new SequentialAction(
                                                reset,
                                                moveToShoot1.build(),
                                                new CommandAction(new SavedVarsCmd(pose2dSupplier)),

                                                new RaceAction(new SmartLineShooterAutoAct(subsystems.shooterSubsystem, subsystems.shooterFeedSubsystem,
                                                        subsystems.sensorSubsystem, subsystems.aimSubsystem, subsystems.spindexerSubsystem,
                                                        subsystems.kickSubsystem, subsystems.intakeSubsystem, subsystems.mecDriveSubsystem, telemetry),
                                                        new CommandAction(new TimeoutCmd(Constants.SHOOTER_TIMEOUT))),
                                                reset,


                                                new ParallelAction(
                                                        new CommandAction(new IntakeLineCmd(subsystems.shooterFeedSubsystem,
                                                                subsystems.intakeSubsystem, subsystems.spindexerSubsystem, subsystems.kickSubsystem)),
                                                        pickup1.build()
                                                ),
                                                new CommandAction(new SavedVarsCmd(pose2dSupplier)),

                                                moveToShoot2.build(),
                                                new CommandAction(new SavedVarsCmd(pose2dSupplier)),

                                                new CommandAction(new StopIntakeCmd(subsystems.intakeSubsystem)),
                                                new RaceAction(new SmartLineShooterAutoAct(subsystems.shooterSubsystem, subsystems.shooterFeedSubsystem,
                                                        subsystems.sensorSubsystem, subsystems.aimSubsystem, subsystems.spindexerSubsystem,
                                                        subsystems.kickSubsystem, subsystems.intakeSubsystem, subsystems.mecDriveSubsystem, telemetry),
                                                        new CommandAction(new TimeoutCmd(Constants.SHOOTER_TIMEOUT))),
                                                reset,

                                                new ParallelAction(
                                                        new CommandAction(new IntakeLineCmd(subsystems.shooterFeedSubsystem,
                                                                subsystems.intakeSubsystem, subsystems.spindexerSubsystem, subsystems.kickSubsystem)),
                                                        pickup2.build()
                                                ),
                                                new CommandAction(new SavedVarsCmd(pose2dSupplier)),

                                                moveToShoot3.build(),
                                                new CommandAction(new SavedVarsCmd(pose2dSupplier)),

                                                new RaceAction(new SmartLineShooterAutoAct(subsystems.shooterSubsystem, subsystems.shooterFeedSubsystem,
                                                        subsystems.sensorSubsystem, subsystems.aimSubsystem, subsystems.spindexerSubsystem,
                                                        subsystems.kickSubsystem, subsystems.intakeSubsystem, subsystems.mecDriveSubsystem, telemetry),
                                                        new CommandAction(new TimeoutCmd(Constants.SHOOTER_TIMEOUT))),
                                                reset,

                                                leave.build(),
                                                new CommandAction(new SavedVarsCmd(pose2dSupplier))
                                        )
                                )
                        ),
                        new CommandAction(new TimeoutCmd(30000))
                )
        );



        SavedVars.startingY = subsystems.mecDriveSubsystem.drive.localizer.getPose().position.y;
        SavedVars.startingX = subsystems.mecDriveSubsystem.drive.localizer.getPose().position.x;
        SavedVars.startingHeading = subsystems.mecDriveSubsystem.drive.localizer.getPose().heading.toDouble();
    }
}*/