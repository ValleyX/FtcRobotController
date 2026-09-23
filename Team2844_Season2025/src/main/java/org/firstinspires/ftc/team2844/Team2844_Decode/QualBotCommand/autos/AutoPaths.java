package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

/**
 * Small helpers for assembling PathChains.
 *
 * <p>Every pose these take is already in Pedro coordinates — build them with
 * {@link org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.PedroConstants#ftcPose}
 * so the field numbers stay in the centre-origin frame the old Road Runner autos
 * were written in. Headings are always read back off the poses rather than
 * written as raw radians, because the two frames do not share a heading origin.
 */
public final class AutoPaths {

    private AutoPaths() {}

    /** Straight line, turning steadily from the start pose's heading to the end pose's. */
    public static PathChain line(Follower follower, Pose start, Pose end) {
        return follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }

    /** Straight line that holds the start pose's heading the whole way. */
    public static PathChain strafe(Follower follower, Pose start, Pose end) {
        return follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setConstantHeadingInterpolation(start.getHeading())
                .build();
    }

    /** Single curve through one control point, turning start heading to end heading. */
    public static PathChain curve(Follower follower, Pose start, Pose control, Pose end) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(start, control, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }

    /**
     * Chains straight segments through a list of poses, interpolating heading
     * across each one.
     */
    public static PathChain through(Follower follower, Pose... poses) {
        if (poses.length < 2) {
            throw new IllegalArgumentException("through() needs at least a start and an end pose");
        }

        PathBuilder builder = follower.pathBuilder();
        for (int i = 0; i < poses.length - 1; i++) {
            builder.addPath(new BezierLine(poses[i], poses[i + 1]))
                    .setLinearHeadingInterpolation(poses[i].getHeading(), poses[i + 1].getHeading());
        }
        return builder.build();
    }

    /**
     * Midpoint between two poses, nudged perpendicular by {@code bulge} inches —
     * a quick way to get a control point that rounds a corner instead of
     * cutting it.
     */
    public static Pose bulgedControl(Pose start, Pose end, double bulge) {
        double midX = (start.getX() + end.getX()) / 2.0;
        double midY = (start.getY() + end.getY()) / 2.0;

        double dx = end.getX() - start.getX();
        double dy = end.getY() - start.getY();
        double length = Math.hypot(dx, dy);
        if (length == 0.0) {
            return new Pose(midX, midY, start.getHeading(), start.getCoordinateSystem());
        }

        // Rotate the segment direction 90 degrees and step along it.
        double offsetX = -dy / length * bulge;
        double offsetY = dx / length * bulge;
        return new Pose(midX + offsetX, midY + offsetY, start.getHeading(), start.getCoordinateSystem());
    }
}
