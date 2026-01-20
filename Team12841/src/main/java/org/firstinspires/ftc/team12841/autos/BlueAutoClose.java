package org.firstinspires.ftc.team12841.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.team12841.pedroPathing.Constants;

@TeleOp
public class BlueAutoClose extends OpMode
{
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    // State Machine
    public enum PathState
    {
        // StartPose - EndPose
        // Drive - Movement
        // Shoot - Scoring
        START_TO_SHOOT,
        SHOOT_PRELOAD,
        INTAKE1,
        INTAKE1_TO_SHOOT,
        SHOOT1_TO_INTAKE2,
        INTAKE2,
        INTAKE2_BUFFER,
        INTAKE2_TO_SHOOT,
        SHOOT2,
        MOVE_AWAY
    }

    PathState pathState;

    // TEMPLATE
    // private final Pose poseName = new Pose(x, y, Math.toRadians(heading));

    private final Pose startPose = new Pose(26.5, 129.1, Math.toRadians(324));
    private final Pose shootPose = new Pose(59.5, 84, Math.toRadians(312));
    private final Pose intake1 = new Pose(16, 84, Math.toRadians(180));
    private final Pose intake2Align = new Pose(48, 60, Math.toRadians(180));
    private final Pose intake2 = new Pose(9, 60, Math.toRadians(180));
    private final Pose endPose = new Pose(14.8, 72, Math.toRadians(270));

    private PathChain drive_StartPos_ShootPos, drive_ShootPos_Intake1Pos, drive_Intake1Pos_ShootPos, drive_ShootPos_Intake2AlignPos, drive_Intake2AlignPos_Intake2Pos, drive_Intake2Pos_ShootPos, drive_ShootPos_EndPos;

    public void buildPaths()
    {
        // StartingPose Coords - Ending Pose Coords

        drive_StartPos_ShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        drive_ShootPos_Intake1Pos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intake1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intake1.getHeading())
                .build();

        drive_Intake1Pos_ShootPos = follower.pathBuilder()
                .addPath(new BezierLine(intake1, shootPose))
                .setLinearHeadingInterpolation(intake1.getHeading(), shootPose.getHeading())
                .build();

        drive_ShootPos_Intake2AlignPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intake2Align))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intake2Align.getHeading())
                .build();

        drive_Intake2AlignPos_Intake2Pos = follower.pathBuilder()
                .addPath(new BezierLine(intake2Align, intake2))
                .setLinearHeadingInterpolation(intake2Align.getHeading(), intake2.getHeading())
                .build();

        drive_Intake2Pos_ShootPos = follower.pathBuilder()
                .addPath(new BezierLine(intake2, shootPose))
                .setLinearHeadingInterpolation(intake2.getHeading(), shootPose.getHeading())
                .build();

        drive_ShootPos_EndPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();

    }

    public void statePathUpdate()
    {
        switch(pathState)
        {
            case START_TO_SHOOT:
                // TODO: Shooter ON with Regression
                follower.followPath(drive_StartPos_ShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);
                        break;

            case SHOOT_PRELOAD:
                // Check if follower is done
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5)
                {
                    // TODO: Shoot, Turn, Shoot, Turn, Shoot, Turn Back
                    setPathState(PathState.INTAKE1);
                }
                break;

            case INTAKE1:
                if(!follower.isBusy())
                {
                    // TODO: Intake ON, TT Spinning
                    follower.followPath(drive_ShootPos_Intake1Pos, true);
                    // TODO: Intake OFF
                    setPathState(PathState.INTAKE1_TO_SHOOT);
                }

            case INTAKE1_TO_SHOOT:
                if(!follower.isBusy())
                {
                    follower.followPath(drive_Intake1Pos_ShootPos, true);
                    // TODO: Shoot thing
                    setPathState(PathState.SHOOT1_TO_INTAKE2);
                }

            case SHOOT1_TO_INTAKE2:
                if(!follower.isBusy())
                {
                    follower.followPath(drive_ShootPos_Intake2AlignPos, true);
                    // TODO: Intake + TT
                    follower.followPath(drive_Intake2AlignPos_Intake2Pos);
                    // TODO: Intake
                    setPathState(PathState.INTAKE2_TO_SHOOT);
                }

            case INTAKE2_TO_SHOOT:
                if(!follower.isBusy())
                {
                    // TODO: Shooter ON
                    follower.followPath(drive_Intake2Pos_ShootPos, true);
                    // TODO: SHoot
                    setPathState(PathState.MOVE_AWAY);
                }

            case MOVE_AWAY:
                if(!follower.isBusy())
                {
                    follower.followPath(drive_ShootPos_EndPos, true);
                }

            default:
                telemetry.addLine("No State");
                break;

        }
    }

    // Helper Function
    public void setPathState(PathState newState)
    {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init()
    {
        pathState = PathState.START_TO_SHOOT;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        // TODO Add other Mechanisms

        buildPaths();
        follower.setPose(startPose);
    }

    public void start()
    {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop()
    {
        // Update Pose
        follower.update();
        statePathUpdate();

        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
    }
}