package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.autos;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Small helpers for assembling paths.
 *
 * <p>Pedro 3 builds paths standalone -- no Follower needed, and no builder to
 * thread through -- so these are mostly thin wrappers that keep the call sites in
 * {@code CloseAutoRoutine} and {@code FarAutoRoutines} reading the way they did
 * before the migration. Heading interpolation now hangs off the path itself
 * ({@code .linear()}, {@code .constant()}, {@code .tangent()}) instead of being
 * set on a builder.
 *
 * <p>Poses are plain FTC-standard field coordinates; Pedro 3 has no coordinate
 * system attached to a {@link Pose}, so there is nothing to convert.
 */
public final class AutoPaths {

    private AutoPaths() {}

    /** Straight line, turning steadily from the start pose's heading to the end pose's. */
    public static Path line(Pose start, Pose end) {
        return Paths.line(start, end).linear(start, end);
    }

    /** Straight line that holds the start pose's heading the whole way. */
    public static Path strafe(Pose start, Pose end) {
        return Paths.line(start, end).constant(start.heading());
    }

    /** Single curve through one control point, turning start heading to end heading. */
    public static Path curve(Pose start, Pose control, Pose end) {
        return Paths.curve(start, control, end).linear(start, end);
    }

    /**
     * Chains straight legs through a list of poses, interpolating heading across
     * each one.
     */
    public static Path through(Pose... poses) {
        if (poses.length < 2) {
            throw new IllegalArgumentException("through() needs at least a start and an end pose");
        }

        Path[] legs = new Path[poses.length - 1];
        for (int i = 0; i < legs.length; i++) {
            legs[i] = line(poses[i], poses[i + 1]);
        }
        return Paths.path(legs);
    }

    /**
     * Midpoint between two poses, nudged perpendicular by {@code bulge} inches --
     * a quick way to get a control point that rounds a corner instead of cutting
     * it.
     */
    public static Pose bulgedControl(Pose start, Pose end, double bulge) {
        double midX = (start.x() + end.x()) / 2.0;
        double midY = (start.y() + end.y()) / 2.0;

        double dx = end.x() - start.x();
        double dy = end.y() - start.y();
        double length = Math.hypot(dx, dy);
        if (length == 0.0) {
            return new Pose(midX, midY, start.heading());
        }

        // Rotate the segment direction 90 degrees and step along it.
        double offsetX = -dy / length * bulge;
        double offsetY = dx / length * bulge;
        return new Pose(midX + offsetX, midY + offsetY, start.heading());
    }
}
