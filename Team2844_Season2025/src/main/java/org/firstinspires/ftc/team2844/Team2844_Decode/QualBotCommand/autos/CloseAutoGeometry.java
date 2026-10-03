package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.autos;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.PedroConstants;

/**
 * Waypoints for the near-goal auto, one set per alliance.
 *
 * <p>These are the same field positions the Road Runner version drove to, in the
 * same centre-origin coordinates -- Pedro 3 takes bare poses, so the numbers
 * below are the original literals with no conversion.
 *
 * <p>The mirroring is now Pedro's job: {@link PedroConstants#RED} is a
 * {@link PoseFactory} carrying {@code mirrorY(0)}, which maps {@code (x, y, h)}
 * to {@code (x, -y, -h)}. Every pose is written once, in blue coordinates, and
 * the factory supplies the alliance. The one place the two sides genuinely differ
 * -- the second pickup run, which was tuned separately on each side -- stays an
 * explicit constructor argument.
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

    private CloseAutoGeometry(PoseFactory alliance,
                              double pickup2EntryY,
                              double pickup2FarY,
                              double pickup2BackY) {
        start = alliance.of(55, 45, 45);
        shoot1 = alliance.of(40, 25, 50);

        pickup1Entry = alliance.of(12, 25, 90);
        pickup1Far = alliance.of(12, 60, 90);
        pickup1Back = alliance.of(12, 45, 90);

        shoot2 = alliance.of(24, 24, SHOOT_HEADING);

        pickup2Entry = alliance.of(-12, pickup2EntryY, 90);
        pickup2Far = alliance.of(-12, pickup2FarY, 90);
        pickup2Back = alliance.of(-12, pickup2BackY, 90);

        exit2 = alliance.of(-12, 35, 90);
        shoot3 = alliance.of(36, 20, SHOOT_HEADING);
    }

    public static CloseAutoGeometry blue() {
        return new CloseAutoGeometry(PedroConstants.BLUE, 18, 66, 50);
    }

    public static CloseAutoGeometry red() {
        return new CloseAutoGeometry(PedroConstants.RED, 24, 66, 50);
    }
}
