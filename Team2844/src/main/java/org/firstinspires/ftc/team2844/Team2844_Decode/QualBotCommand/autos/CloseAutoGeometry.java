package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.autos;

import com.pedropathing.geometry.Pose;

import static org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.PedroConstants.ftcPose;

/**
 * Waypoints for the near-goal auto, one set per alliance.
 *
 * <p>These are the same field positions the Road Runner version drove to,
 * written in the same centre-origin coordinates. The red set is very nearly the
 * blue set mirrored across y, but not exactly — the second pickup run was tuned
 * separately on each side — so both are spelled out rather than derived.
 *
 * <p>Angles that look arbitrary come from the original: {@code PI/3.5} is the
 * 51.4 degree shooting heading, and the first shot sits 5 degrees off the
 * 45 degree start heading.
 */
public class CloseAutoGeometry {

    private static final double SHOOT_HEADING = Math.toDegrees(Math.PI / 3.5);

    /** Where the robot is placed before the match. */
    public final Pose start;

    /** First shooting position. */
    public final Pose shoot1;

    /** Run down the first ball line: enter, sweep to the far end, then back off. */
    public final Pose pickup1Entry;
    public final Pose pickup1Far;
    public final Pose pickup1Back;

    /** Second shooting position. */
    public final Pose shoot2;

    /** Second ball line. */
    public final Pose pickup2Entry;
    public final Pose pickup2Far;
    public final Pose pickup2Back;

    /** Staging pose on the way out of the second line. */
    public final Pose exit2;

    /** Third and final shooting position. */
    public final Pose shoot3;

    private CloseAutoGeometry(int sign,
                              double pickup2EntryY,
                              double pickup2FarY,
                              double pickup2BackY) {
        start = pose(55, 45, 45, sign);
        shoot1 = pose(40, 25, 50, sign);

        pickup1Entry = pose(12, 25, 90, sign);
        pickup1Far = pose(12, 60, 90, sign);
        pickup1Back = pose(12, 45, 90, sign);

        shoot2 = pose(24, 24, SHOOT_HEADING, sign);

        pickup2Entry = pose(-12, pickup2EntryY, 90, sign);
        pickup2Far = pose(-12, pickup2FarY, 90, sign);
        pickup2Back = pose(-12, pickup2BackY, 90, sign);

        exit2 = pose(-12, 35, 90, sign);
        shoot3 = pose(36, 20, SHOOT_HEADING, sign);
    }

    /** Mirrors y and heading for the red side; blue passes {@code sign = 1}. */
    private static Pose pose(double x, double y, double headingDegrees, int sign) {
        return ftcPose(x, y * sign, Math.toRadians(headingDegrees * sign));
    }

    public static CloseAutoGeometry blue() {
        return new CloseAutoGeometry(1, 18, 66, 50);
    }

    public static CloseAutoGeometry red() {
        return new CloseAutoGeometry(-1, 24, 66, 50);
    }
}
