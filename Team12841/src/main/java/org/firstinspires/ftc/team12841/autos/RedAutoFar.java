package org.firstinspires.ftc.team12841.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.team12841.pedroPathing.Constants;

@Autonomous(name = "Red Far", group = "Autonomous")
@Configurable // Panels
public class RedAutoFar extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Timer pathTimer; // Timer for path state machine
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        pathTimer = new Timer();
        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain STARTTOPRELOAD;
        public PathChain INTAKE1ALIGN;
        public PathChain INTAKE1;
        public PathChain INTAKE1TOSHOOT;
        public PathChain INTAKE2ALIGN;
        public PathChain INTAKE2;
        public PathChain INTAKE2TOSHOOT;
        public PathChain INTAKE3ALIGN;
        public PathChain INTAKE3;
        public PathChain MOVEAWAY;

        public Paths(Follower follower) {
            STARTTOPRELOAD = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(79.000, 9.000), new Pose(84.000, 14.900))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(247))
                    .build();

            INTAKE1ALIGN = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(84.000, 14.900), new Pose(84.000, 35.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(247), Math.toRadians(0))
                    .build();

            INTAKE1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(84.000, 35.000), new Pose(120.800, 35.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            INTAKE1TOSHOOT = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(120.800, 35.000), new Pose(84.000, 14.900))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(247))
                    .build();

            INTAKE2ALIGN = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(84.000, 14.900), new Pose(84.000, 60.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(247), Math.toRadians(0))
                    .build();

            INTAKE2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(84.000, 60.000), new Pose(120.800, 60.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            INTAKE2TOSHOOT = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(120.800, 60.000), new Pose(84.000, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))
                    .build();

            INTAKE3ALIGN = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(84.000, 84.000), new Pose(101.700, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))
                    .build();

            INTAKE3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(101.700, 84.000), new Pose(125.000, 84.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            MOVEAWAY = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(125.000, 84.000), new Pose(125.000, 70.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.STARTTOPRELOAD, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;
            case 2:
                setPathState(3);
                break;
            case 3:
                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(paths.INTAKE1ALIGN, true);
                }
                setPathState(5);
                break;
            case 5:
                if (!follower.isBusy()) {
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.INTAKE1, true);
                }
                setPathState(7);
                break;
            case 7:
                if (!follower.isBusy()) {
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.INTAKE1TOSHOOT, true);
                }
                setPathState(9);
                break;
            case 9:
                if (!follower.isBusy()) {
                    setPathState(10);
                }
                break;
            case 10:
                setPathState(11);
                break;
            case 11:
                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(paths.INTAKE2ALIGN, true);
                }
                setPathState(13);
                break;
            case 13:
                if (!follower.isBusy()) {
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    follower.followPath(paths.INTAKE2, true);
                }
                setPathState(15);
                break;
            case 15:
                if (!follower.isBusy()) {
                    setPathState(16);
                }
                break;
            case 16:
                if (!follower.isBusy()) {
                    follower.followPath(paths.INTAKE2TOSHOOT, true);
                }
                setPathState(17);
                break;
            case 17:
                if (!follower.isBusy()) {
                    setPathState(18);
                }
                break;
            case 18:
                setPathState(19);
                break;
            case 19:
                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    setPathState(20);
                }
                break;
            case 20:
                if (!follower.isBusy()) {
                    follower.followPath(paths.INTAKE3ALIGN, true);
                }
                setPathState(21);
                break;
            case 21:
                if (!follower.isBusy()) {
                    setPathState(22);
                }
                break;
            case 22:
                if (!follower.isBusy()) {
                    follower.followPath(paths.INTAKE3, true);
                }
                setPathState(23);
                break;
            case 23:
                if (!follower.isBusy()) {
                    setPathState(24);
                }
                break;
            case 24:
                if (!follower.isBusy()) {
                    follower.followPath(paths.MOVEAWAY, true);
                }
                setPathState(25);
                break;
            case 25:
                if (!follower.isBusy()) {
                    setPathState(26);
                }
                break;
            case 26:
                requestOpModeStop();
                pathState = -1;
                break;
        }
        return pathState;
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}
