package org.firstinspires.ftc.team12841.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.team12841.pedroPathing.Constants;

@Autonomous(name = "Blue Close", group = "Autonomous")
@Configurable // Panels
public class BlueAutoClose extends OpMode {

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

        public PathChain STARTTOSHOOT;
        public PathChain INTAKE1;
        public PathChain INTAKE1TOSHOOT;
        public PathChain SHOOT1TOINTAKE2ALIGN;
        public PathChain INTAKE2;
        public PathChain INTAKE2TOSHOOT;
        public PathChain MOVEAWAY;

        public Paths(Follower follower) {
            STARTTOSHOOT = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(26.600, 129.000),
                                    new Pose(40.368, 110.102),
                                    new Pose(36.187, 115.940),
                                    new Pose(59.500, 84.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(324), Math.toRadians(312))
                    .build();

            INTAKE1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(59.500, 84.000), new Pose(16.000, 84.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            INTAKE1TOSHOOT = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(16.000, 84.000), new Pose(59.500, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(312))
                    .setReversed()
                    .build();

            SHOOT1TOINTAKE2ALIGN = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(59.500, 84.000), new Pose(43.000, 60.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(312), Math.toRadians(180))
                    .build();

            INTAKE2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(43.000, 60.000), new Pose(14.000, 60.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            INTAKE2TOSHOOT = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(14.000, 60.000), new Pose(59.500, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(312))
                    .build();

            MOVEAWAY = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(59.500, 84.000), new Pose(60.000, 36.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(312), Math.toRadians(180))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.STARTTOSHOOT, true);
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
                follower.followPath(paths.INTAKE1, true);
                setPathState(5);
                break;
            case 5:
                if (!follower.isBusy()) {
                    setPathState(6);
                }
                break;
            case 6:
                follower.followPath(paths.INTAKE1TOSHOOT, true);
                setPathState(7);
                break;
            case 7:
                if (!follower.isBusy()) {
                    setPathState(8);
                }
                break;
            case 8:
                setPathState(9);
                break;
            case 9:
                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    setPathState(10);
                }
                break;
            case 10:
                follower.followPath(paths.SHOOT1TOINTAKE2ALIGN, true);
                setPathState(11);
                break;
            case 11:
                if (!follower.isBusy()) {
                    setPathState(12);
                }
                break;
            case 12:
                follower.followPath(paths.INTAKE2, true);
                setPathState(13);
                break;
            case 13:
                if (!follower.isBusy()) {
                    setPathState(14);
                }
                break;
            case 14:
                follower.followPath(paths.INTAKE2TOSHOOT, true);
                setPathState(15);
                break;
            case 15:
                if (!follower.isBusy()) {
                    setPathState(16);
                }
                break;
            case 16:
                setPathState(17);
                break;
            case 17:
                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    setPathState(18);
                }
                break;
            case 18:
                follower.followPath(paths.MOVEAWAY, true);
                setPathState(19);
                break;
            case 19:
                if (!follower.isBusy()) {
                    setPathState(20);
                }
                break;
            case 20:
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
