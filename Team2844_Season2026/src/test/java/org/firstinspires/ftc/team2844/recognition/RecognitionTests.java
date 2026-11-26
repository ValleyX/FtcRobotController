package org.firstinspires.ftc.team2844.recognition;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.firstinspires.ftc.team2844global.Ball;
import org.firstinspires.ftc.team2844global.GlobalConstants;
import org.firstinspires.ftc.team2844global.MatchInfo;
import org.junit.BeforeClass;
import org.junit.FixMethodOrder;
import org.junit.Test;
import org.junit.runners.MethodSorters;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Set;

import static org.firstinspires.ftc.team2844.recognition.RecognitionSim.*;

/**
 * Desktop tests for the HuskyLens ball recognition: HuskySubsystem + Ball.
 * They run the real robot classes on your computer, with a fake HuskyLens (see RecognitionSim).
 *
 * <p>HOW TO RUN
 * <ul>
 *     <li>Android Studio: click the green arrow next to this class (all tests) or next to one test.
 *     The report for each test is in the Run window when you click on that test.</li>
 *     <li>Command line, from the repository folder:
 *     <pre>./gradlew :Team2844_Season2026:testDebugUnitTest --tests "*RecognitionTests*" --rerun</pre>
 *     One test: <pre>--tests "*RecognitionTests.s06*"</pre></li>
 * </ul>
 *
 * <p>A test goes red when one of its checks fails; the message lists which ones.
 * Everything else the tests print is measurements, not pass/fail.
 * Camera behavior the tests can't know (frame rate, read time) is guessed at the top of RecognitionSim.
 */
@FixMethodOrder(MethodSorters.NAME_ASCENDING)
public class RecognitionTests {
    /** Robot near the audience wall, facing the back wall. Most scenarios use this. */
    private static final Pose FACING_BACK = new Pose(72, 24, Math.PI / 2);

    @BeforeClass
    public static void setUp() throws Exception {
        RecognitionSim.setup();
        printSettings();
    }

    /* =============================== 1. camera math =============================== */

    @Test
    public void s01_cameraMath() {
        Checks c = new Checks("1. Camera math: HuskySubsystem vs an independent ray-cast, every pixel");
        double worst = 0;
        int n = 0;
        for (int id : new int[]{POLLEN, BLUE_N})
            for (int px = 0; px <= (int) IMG_W; px++)
                for (int py = (int) TOP_CUT; py <= (int) IMG_H; py++) {
                    double[] ref = raycast(px, py, radius(id));
                    double[] got = codeRel(block(id, px, py, 10, 10));
                    if (ref == null) { c.check(false, "ray-cast misses the floor at " + px + "," + py); continue; }
                    double e = Math.hypot(ref[0] - got[0], ref[1] - got[1]);
                    worst = Math.max(worst, e);
                    n++;
                    c.check(e < 1e-6, String.format("pixel (%d,%d) %s: code (%.4f, %.4f) ray (%.4f, %.4f)", px, py, color(id), got[0], got[1], ref[0], ref[1]));
                }
        System.out.printf("  %d pixels x 2 ball sizes checked, biggest difference %.1e in%n", n / 2, worst);

        double[] nearL = raycast(0, (int) IMG_H, R_POLLEN), farL = raycast(0, (int) TOP_CUT, R_POLLEN);
        double[] nearC = raycast((int) (IMG_W / 2), (int) IMG_H, R_POLLEN), farC = raycast((int) (IMG_W / 2), (int) TOP_CUT, R_POLLEN);
        System.out.println("  Where a pollen center can be seen (inches from the robot center):");
        System.out.printf("    bottom row of the image:   %.1f forward, +/-%.1f to the sides%n", nearC[0], Math.abs(nearL[1]));
        System.out.printf("    TOP_CUTTOFF row (y=%d):    %.1f forward, +/-%.1f to the sides%n", (int) TOP_CUT, farC[0], Math.abs(farL[1]));
        System.out.println("  Size of one pixel on the floor (center column):");
        for (int py : new int[]{240, 200, 160, 120, 90, 70}) {
            if (py < TOP_CUT || py > IMG_H) continue;
            double[] a = raycast(160, py, R_POLLEN), b = raycast(160, py - 1, R_POLLEN), d = raycast(161, py, R_POLLEN);
            System.out.printf("    row %3d: %5.1f in forward   1 px up = %.2f in farther   1 px right = %.2f in%n", py, a[0], b[0] - a[0], d[1] - a[1]);
        }
        c.finish();
    }

    /* =============================== 2. rounding + edges =============================== */

    @Test
    public void s02_pixelRounding() {
        Checks c = new Checks("2. Error from whole-pixel rounding, and from balls cut off at the image edge (grid every 0.25 in)");
        double[] edges = {0, 30, 45, 60, 80, 100, 1e9};
        String[] names = {"< 30 in", "30-45", "45-60", "60-80", "80-100", "> 100"};
        Stats[] bands = new Stats[6];
        for (int i = 0; i < 6; i++) bands[i] = new Stats();
        Stats clipBottom = new Stats(), clipSide = new Stats();
        double[] worstClip = null;
        double worstClipErr = 0;
        for (int id : new int[]{POLLEN, BLUE_N})
            for (double fwd = 10; fwd <= 120; fwd += 0.25)
                for (double right = -60; right <= 60; right += 0.25) {
                    double[] p = projectRel(fwd, right, radius(id));
                    if (p == null) continue;
                    int[] bx = boxFor(id, p, 0, 0);
                    if (bx == null || bx[2] < TOP_CUT) continue;
                    double[] got = codeRel(toBlock(bx));
                    double e = Math.hypot(got[0] - fwd, got[1] - right);
                    if (bx[5] == 1) {
                        boolean bottom = p[1] + p[3] > IMG_H;
                        (bottom ? clipBottom : clipSide).add(e);
                        if (e > worstClipErr) { worstClipErr = e; worstClip = new double[]{fwd, right, got[0], got[1]}; }
                        continue;
                    }
                    int k = 0;
                    while (fwd >= edges[k + 1]) k++;
                    bands[k].add(e);
                    c.check(e < 2.0, String.format("fully visible ball at fwd %.2f right %.2f read %.2f in off", fwd, right, e));
                }
        System.out.println("  Fully on screen, error in inches by distance forward from the robot center:");
        for (int i = 0; i < 6; i++) if (bands[i].n > 0) System.out.printf("    %-8s %s%n", names[i], bands[i].line());
        System.out.println("  Cut off at the bottom edge: " + clipBottom.line());
        System.out.println("  Cut off at a side edge:     " + clipSide.line());
        if (worstClip != null)
            System.out.printf("  Worst cut-off ball: really at fwd %.1f right %.1f, code says fwd %.1f right %.1f%n", worstClip[0], worstClip[1], worstClip[2], worstClip[3]);
        c.finish();
    }

    /* =============================== 3. field transform =============================== */

    @Test
    public void s03_fieldTransform() {
        Checks c = new Checks("3. Field position through getVisibleBalls at 3000 random robot poses and headings");
        Random rnd = new Random(3);
        MatchInfo.alliance = GlobalConstants.BLUE_ALLIANCE;
        Stats math = new Stats(), truth = new Stats();
        int frames = 0;
        for (int trial = 0; trial < 3000; trial++) {
            Pose robot = new Pose(15 + rnd.nextDouble() * 114, 15 + rnd.nextDouble() * 114, (rnd.nextDouble() * 2 - 1) * Math.PI);
            int want = 1 + rnd.nextInt(5);
            List<int[]> boxes = new ArrayList<>();
            List<double[]> truths = new ArrayList<>();
            for (int i = 0; i < want; i++) {
                double fwd = 25 + rnd.nextDouble() * 65;
                double half = 0.8 * (fwd - FWD_OFF) * Math.tan(Math.toRadians(HFOV) / 2);
                double right = (rnd.nextDouble() * 2 - 1) * half;
                int id = rnd.nextBoolean() ? POLLEN : BLUE_N;
                double[] f = toField(robot, fwd, right);
                if (f[0] < 0 || f[0] > FIELD || f[1] < 0 || f[1] > FIELD) continue;
                double[] p = projectRel(fwd, right, radius(id));
                if (p == null) continue;
                int[] bx = boxFor(id, p, 0, 0);
                if (bx == null || bx[5] == 1 || bx[2] < TOP_CUT) continue;
                boxes.add(bx);
                truths.add(f);
            }
            if (boxes.isEmpty()) continue;
            HuskyLens.Block[] frame = toBlocks(boxes);
            List<Ball> got = visible(MatchInfo.alliance, robot, frame);
            frames++;

            List<double[]> mine = new ArrayList<>();
            for (HuskyLens.Block b : frame) {
                double[] rel = codeRel(b);
                double[] f = toField(robot, rel[0], rel[1]);
                if (f[0] >= 0 && f[0] <= FIELD && f[1] >= 0 && f[1] <= FIELD) mine.add(f);
            }
            c.check(got.size() == mine.size(), "trial " + trial + ": getVisibleBalls returned " + got.size() + " balls, expected " + mine.size());
            for (int i = 1; i < got.size(); i++)
                c.check(robot.distance(got.get(i - 1).getPose()) <= robot.distance(got.get(i).getPose()) + 1e-9, "trial " + trial + ": not sorted closest first");
            for (Ball b : got) {
                double bm = 1e9, bt = 1e9;
                for (double[] m : mine) bm = Math.min(bm, Math.hypot(m[0] - b.getPose().x(), m[1] - b.getPose().y()));
                for (double[] t : truths) bt = Math.min(bt, Math.hypot(t[0] - b.getPose().x(), t[1] - b.getPose().y()));
                math.add(bm);
                truth.add(bt);
                c.check(bm < 1e-6, String.format("trial %d: field transform off by %.4f in", trial, bm));
            }
        }
        System.out.printf("  %d frames, %d balls%n", frames, math.n);
        System.out.println("  getVisibleBalls vs my own rotation of the same pixels: " + math.line());
        System.out.println("  getVisibleBalls vs where the ball really is:            " + truth.line());
        c.finish();
    }

    /* =============================== 4. filters + sort =============================== */

    @Test
    public void s04_filters() {
        Checks c = new Checks("4. Filters and sorting");
        Pose robot = FACING_BACK;
        for (int alliance : new int[]{GlobalConstants.BLUE_ALLIANCE, GlobalConstants.RED_ALLIANCE, GlobalConstants.NO_ALLIANCE}) {
            List<Integer> kept = new ArrayList<>();
            for (int id = 0; id <= 6; id++)
                if (!visible(alliance, robot, block(id, 160, 160, 20, 20)).isEmpty()) kept.add(id);
            String name = alliance == GlobalConstants.BLUE_ALLIANCE ? "BLUE" : alliance == GlobalConstants.RED_ALLIANCE ? "RED" : "NO_ALLIANCE";
            System.out.println("  alliance " + name + ": IDs kept = " + kept);
            if (alliance == GlobalConstants.BLUE_ALLIANCE) c.check(kept.equals(listOf(POLLEN, BLUE_N)), "BLUE should keep only pollen and blue nectar, kept " + kept);
            if (alliance == GlobalConstants.RED_ALLIANCE) c.check(kept.equals(listOf(POLLEN, RED_N)), "RED should keep only pollen and red nectar, kept " + kept);
        }

        c.check(visible(0, robot, block(POLLEN, 160, (int) TOP_CUT - 1, 20, 20)).isEmpty(), "y = TOP_CUTTOFF-1 should be removed");
        c.check(visible(0, robot, block(POLLEN, 160, (int) TOP_CUT, 20, 20)).size() == 1, "y = TOP_CUTTOFF should be kept");
        c.check(visible(0, robot).isEmpty(), "no blocks should give no balls");

        // outside the field
        int[] bx = boxFor(POLLEN, projectRel(40, 0, R_POLLEN), 0, 0);
        c.check(visible(0, new Pose(10, 72, Math.PI), toBlock(bx)).isEmpty(), "ball 30 in past the red wall should be removed");
        c.check(visible(0, new Pose(72, 134, Math.PI / 2), toBlock(bx)).isEmpty(), "ball 30 in past the back wall should be removed");
        c.check(visible(0, robot, toBlock(bx)).size() == 1, "same ball inside the field should be kept");

        // sorting
        Random rnd = new Random(4);
        for (int trial = 0; trial < 500; trial++) {
            List<HuskyLens.Block> list = new ArrayList<>();
            for (int i = 0; i < 6; i++) list.add(block(POLLEN, 20 + rnd.nextInt(280), (int) TOP_CUT + rnd.nextInt((int) (IMG_H - TOP_CUT)), 20, 20));
            List<Ball> got = visible(0, robot, list.toArray(new HuskyLens.Block[0]));
            for (int i = 1; i < got.size(); i++)
                c.check(robot.distance(got.get(i - 1).getPose()) <= robot.distance(got.get(i).getPose()) + 1e-9, "sort trial " + trial);
        }
        System.out.println("  TOP_CUTTOFF, empty frame, outside-field and 500 sort trials checked");
        c.finish();
    }

    /* =============================== 5. tracking =============================== */

    @Test
    public void s05_tracking() {
        Checks c = new Checks("5. Tracking (setTrackIDs)");
        Random rnd = new Random(5);

        // the MAX_MATCH_DISTANCE boundary
        for (double move : new double[]{MAX_MATCH - 0.1, MAX_MATCH + 0.1}) {
            Ball o = new Ball(new Pose(50, 50), POLLEN), nb = new Ball(new Pose(50 + move, 50), POLLEN);
            int own = nb.getTrackID();
            husky.setTrackIDs(new ArrayList<>(listOf(nb)), listOf(o));
            boolean matched = nb.getTrackID() == o.getTrackID();
            c.check(matched == (move < MAX_MATCH), "moved " + move + " in: matched=" + matched);
            if (!matched) c.check(nb.getTrackID() == own, "unmatched ball should keep its own ID");
        }
        // color
        {
            Ball o = new Ball(new Pose(10, 10), POLLEN);
            Ball nectar = new Ball(new Pose(10.5, 10), BLUE_N), pollen = new Ball(new Pose(13, 10), POLLEN);
            int nectarOwn = nectar.getTrackID();
            husky.setTrackIDs(new ArrayList<>(listOf(nectar, pollen)), listOf(o));
            c.check(pollen.getTrackID() == o.getTrackID() && nectar.getTrackID() == nectarOwn, "color must match");
        }
        // empty lists
        husky.setTrackIDs(new ArrayList<Ball>(), listOf(new Ball(new Pose(1, 1), POLLEN)));
        husky.setTrackIDs(new ArrayList<>(listOf(new Ball(new Pose(1, 1), POLLEN))), new ArrayList<Ball>());
        c.check(true, "empty lists");

        // spread-out balls
        int trials = 20000, bad = 0;
        for (int t = 0; t < trials; t++) {
            int n = 1 + rnd.nextInt(6);
            List<double[]> spots = spaced(rnd, n + 2, 8.0);
            if (spots.size() < n + 2) continue;
            List<Ball> olds = new ArrayList<>(), news = new ArrayList<>();
            Map<Ball, Integer> expect = new HashMap<>();
            for (int i = 0; i < n; i++) {
                int id = rnd.nextBoolean() ? POLLEN : BLUE_N;
                Ball o = new Ball(new Pose(spots.get(i)[0], spots.get(i)[1]), id);
                olds.add(o);
                if (rnd.nextDouble() < 0.2) continue;
                double a = rnd.nextDouble() * 2 * Math.PI, d = rnd.nextDouble() * 3.0;
                Ball nb = new Ball(new Pose(o.getPose().x() + d * Math.cos(a), o.getPose().y() + d * Math.sin(a)), id);
                news.add(nb);
                expect.put(nb, o.getTrackID());
            }
            int added = rnd.nextInt(3);
            for (int i = 0; i < added; i++) {
                double[] sp = spots.get(n + i);
                Ball nb = new Ball(new Pose(sp[0], sp[1]), rnd.nextBoolean() ? POLLEN : BLUE_N);
                news.add(nb);
                expect.put(nb, nb.getTrackID());
            }
            Collections.shuffle(news, rnd);
            Collections.shuffle(olds, rnd);
            husky.setTrackIDs(news, olds);
            boolean ok = true;
            Set<Integer> seen = new HashSet<>();
            for (Ball nb : news) {
                if (nb.getTrackID() != expect.get(nb)) ok = false;
                if (!seen.add(nb.getTrackID())) ok = false;
            }
            c.check(ok, "spread-out tracking trial " + t);
            if (!ok) bad++;
        }
        System.out.printf("  %d random frames, 1-6 balls 8+ in apart, each moves up to 3 in, 20%% blocked, 0-2 new: %d wrong%n", trials, bad);

        // close together: how often IDs swap
        System.out.println("  Two same-color balls close together, each moving up to 3 in in a random direction (5000 tries each):");
        double[][] bandsIn = {{2.8, 4}, {4, 5}, {5, 6}, {6, 8}, {8, 11}};
        for (double[] band : bandsIn) {
            int swaps = 0, steals = 0, tries = 5000;
            for (int t = 0; t < tries; t++) {
                double gap = band[0] + rnd.nextDouble() * (band[1] - band[0]);
                double ang = rnd.nextDouble() * 2 * Math.PI;
                Ball a = new Ball(new Pose(72, 72), POLLEN);
                Ball b = new Ball(new Pose(72 + gap * Math.cos(ang), 72 + gap * Math.sin(ang)), POLLEN);
                double a1 = rnd.nextDouble() * 2 * Math.PI, d1 = rnd.nextDouble() * 3, a2 = rnd.nextDouble() * 2 * Math.PI, d2 = rnd.nextDouble() * 3;
                Ball na = new Ball(new Pose(a.getPose().x() + d1 * Math.cos(a1), a.getPose().y() + d1 * Math.sin(a1)), POLLEN);
                Ball nbb = new Ball(new Pose(b.getPose().x() + d2 * Math.cos(a2), b.getPose().y() + d2 * Math.sin(a2)), POLLEN);
                husky.setTrackIDs(new ArrayList<>(listOf(na, nbb)), listOf(a, b));
                if (na.getTrackID() != a.getTrackID() || nbb.getTrackID() != b.getTrackID()) swaps++;
                // same thing but b is blocked: does a's new ball get b's ID?
                Ball na2 = new Ball(na.getPose(), POLLEN);
                husky.setTrackIDs(new ArrayList<>(listOf(na2)), listOf(a, b));
                if (na2.getTrackID() == b.getTrackID()) steals++;
            }
            System.out.printf("    %4.1f-%4.1f in apart: IDs swapped %5.1f%%   with one blocked, the other took its ID %5.1f%%%n",
                    band[0], band[1], 100.0 * swaps / tries, 100.0 * steals / tries);
        }
        c.finish();
    }

    private static List<double[]> spaced(Random rnd, int n, double minGap) {
        List<double[]> out = new ArrayList<>();
        int tries = 0;
        while (out.size() < n && tries++ < 20000) {
            double x = 5 + rnd.nextDouble() * 134, y = 5 + rnd.nextDouble() * 134;
            boolean ok = true;
            for (double[] o : out) if (Math.hypot(o[0] - x, o[1] - y) < minGap) { ok = false; break; }
            if (ok) out.add(new double[]{x, y});
        }
        return out;
    }

    /* =============================== 6. prediction =============================== */

    private static Pose predict(int id, double x, double y, double vx, double vy) {
        return new Ball(new Pose(x, y), id, Vector2D.cartesian(vx, vy)).predictEndPose();
    }

    @Test
    public void s06_prediction() {
        Checks c = new Checks("6. predictEndPose vs a fine-step physics sim (same slowdown and bounce rules)");
        Object[][] cases = {
                {"not moving", POLLEN, 72, 30, 0, 0},
                {"slow +x, open floor", POLLEN, 20, 20, 10, 0},
                {"+x into blue wall", POLLEN, 130, 20, 30, 0},
                {"-x into red wall", POLLEN, 14, 20, -30, 0},
                {"+y into back wall", POLLEN, 20, 125, 0, 30},
                {"-y into audience wall", POLLEN, 20, 14, 0, -30},
                {"into blue hive, left face", POLLEN, 85, 72, 30, 0},
                {"into blue hive, right face", POLLEN, 106, 72, -30, 0},
                {"into blue hive, top end", POLLEN, 95.7, 100, 0, -30},
                {"into red hive, bottom end", POLLEN, 48.3, 40, 0, 30},
                {"into blue y-flower", POLLEN, 125, 96, 30, 0},
                {"into red x-flower", POLLEN, 48, 125, 0, 30},
                {"diagonal into blue wall", POLLEN, 125, 30, 20, 20},
                {"diagonal into a corner", POLLEN, 130, 130, 30, 30},
                {"nectar into blue wall", BLUE_N, 130, 40, 30, 0},
                {"clips the hive corner", POLLEN, 85, 50, 30, 5},
                {"fast along the audience wall", POLLEN, 20, 3, 60, 0},
                {"very slow", POLLEN, 72, 72, 0.5, 0.2},
        };
        //the code and the sim can disagree a tiny bit about exactly where a bounce lands
        double cornerTol = 0.3;
        System.out.printf("  Balls that bounce within CORNER_MARGIN (%.1f in) of a corner should be dropped (predictEndPose returns null).%n", CORNER_MARGIN);
        System.out.println("  Hand-picked cases (checked: kept balls within 2 in, dropped balls really come near a corner):");
        System.out.printf("  %-30s %-14s %-16s %-16s %-7s %s%n", "case", "start", "real end", "predicted", "error", "bounces");
        for (Object[] cs : cases) {
            String name = (String) cs[0];
            int id = (Integer) cs[1];
            double x = ((Number) cs[2]).doubleValue(), y = ((Number) cs[3]).doubleValue();
            double vx = ((Number) cs[4]).doubleValue(), vy = ((Number) cs[5]).doubleValue();
            SimBall s = new SimBall(id, x, y, vx, vy, false);
            Pose got;
            try {
                got = predict(id, x, y, vx, vy);
            } catch (Throwable t) {
                c.check(false, name + " threw " + t);
                System.out.println("  " + name + " THREW " + t);
                continue;
            }
            if (got == null) {
                System.out.printf("  %-30s %-14s %-16s %-16s %-7s %d   (real path bounces %.2f in from a corner)%n",
                        name, p2(x, y), p2(s.x, s.y), "dropped", "", s.bounces, s.cornerDist);
                c.check(s.cornerDist < CORNER_MARGIN + cornerTol, name + ": dropped, but the real path never comes near a corner");
                continue;
            }
            double e = Math.hypot(got.x() - s.x, got.y() - s.y);
            System.out.printf("  %-30s %-14s %-16s %-16s %5.2f   %d%s%n", name, p2(x, y), p2(s.x, s.y), p2(got.x(), got.y()), e, s.bounces,
                    e > 2 ? "   <-- off by more than 2 in" : "");
            c.check(e < 2.0, name + ": predicted " + p2(got.x(), got.y()) + " real " + p2(s.x, s.y));
        }

        Random rnd = new Random(6);
        Stats[] byBounce = {new Stats(), new Stats(), new Stats()};
        int threw = 0, missedCorner = 0;
        List<String> exceptions = new ArrayList<>();
        int[] outsideByBounce = new int[3], obstacleByBounce = new int[3], droppedByBounce = new int[3], countByBounce = new int[3];
        List<double[]> worst = new ArrayList<>();
        for (int t = 0; t < 5000; t++) {
            int id = rnd.nextInt(4) == 0 ? BLUE_N : POLLEN;
            double r = radius(id), x, y;
            do { x = rnd.nextDouble() * FIELD; y = rnd.nextDouble() * FIELD; } while (!clearSpot(x, y, r, 0.3));
            double speed = rnd.nextDouble() * 80, ang = rnd.nextDouble() * 2 * Math.PI;
            double vx = speed * Math.cos(ang), vy = speed * Math.sin(ang);
            SimBall s = new SimBall(id, x, y, vx, vy, false);
            Pose got;
            try {
                got = predict(id, x, y, vx, vy);
            } catch (Throwable ex) {
                threw++;
                String what = String.format("random ball at %s v=(%.1f, %.1f) threw %s", p2(x, y), vx, vy, ex);
                StackTraceElement[] st = ex.getStackTrace();
                for (StackTraceElement el : st) {
                    if (el.getClassName().startsWith("org.firstinspires")) { what += "  at " + el; break; }
                }
                if (exceptions.size() < 5) exceptions.add(what);
                c.check(false, what);
                continue;
            }
            int k = Math.min(2, s.bounces);
            countByBounce[k]++;
            String ball = String.format("ball at %s v=(%.1f, %.1f)", p2(x, y), vx, vy);
            if (got == null) {
                droppedByBounce[k]++;
                c.check(s.cornerDist < CORNER_MARGIN + cornerTol,
                        String.format("%s dropped, but its real path only comes %.2f in from a corner", ball, s.cornerDist));
                continue;
            }
            double e = Math.hypot(got.x() - s.x, got.y() - s.y);
            byBounce[k].add(e);
            boolean out = outsideField(got.x(), got.y(), r), in = insideObstacle(got.x(), got.y(), r);
            if (out) outsideByBounce[k]++;
            if (in) obstacleByBounce[k]++;
            if (k == 0) c.check(e < 0.05, String.format("no bounce, %s: off by %.3f", ball, e));
            if (s.cornerDist > CORNER_MARGIN + cornerTol)
                c.check(e < 2.0, String.format("%s, %d bounces: predicted %s, real end %s", ball, s.bounces, p2(got.x(), got.y()), p2(s.x, s.y)));
            if (s.cornerDist < CORNER_MARGIN - cornerTol) missedCorner++;
            c.check(!out, String.format("%s predicted outside the field at %s", ball, p2(got.x(), got.y())));
            c.check(!in, String.format("%s predicted inside an obstacle at %s", ball, p2(got.x(), got.y())));
            worst.add(new double[]{e, x, y, vx, vy, s.x, s.y, got.x(), got.y(), s.bounces});
        }
        System.out.println("  5000 random balls, 0-80 in/s, random spots and directions. Error vs the sim for the balls that are kept, by how many times the real ball bounces:");
        System.out.println("  (checked: kept balls within 2 in unless the real path comes near a corner, none outside the field or inside an obstacle)");
        String[] names = {"0 bounces", "1 bounce", "2+ bounces"};
        for (int k = 0; k < 3; k++)
            System.out.printf("    %-11s dropped %4d of %4d   kept: %s   ends outside field %d, inside an obstacle %d%n",
                    names[k], droppedByBounce[k], countByBounce[k], byBounce[k].line(), outsideByBounce[k], obstacleByBounce[k]);
        System.out.printf("  kept even though the real path bounces off a corner: %d%n", missedCorner);
        System.out.println("    (CollisionCircle.collisionInPathPose checks every 1 in, so it can step over a corner the ball only grazes)");
        System.out.printf("  threw an exception: %d%n", threw);
        for (String e : exceptions) System.out.println("    " + e);
        worst.sort((a, b) -> Double.compare(b[0], a[0]));
        System.out.println("  Worst 8:");
        for (int i = 0; i < Math.min(8, worst.size()); i++) {
            double[] w = worst.get(i);
            System.out.printf("    error %6.1f  start %-14s v=(%5.1f, %5.1f)  real end %-14s predicted %-14s bounces %d%n",
                    w[0], p2(w[1], w[2]), w[3], w[4], p2(w[5], w[6]), p2(w[7], w[8]), (int) w[9]);
        }
        c.finish();
    }

    /* =============================== 7. speed at last sighting =============================== */

    @Test
    public void s07_speedAtLastSighting() throws Exception {
        Checks c = new Checks("7. setPose(Ball): speed at the last sighting (real timers)");
        double[][] cases = {{10, 0.5}, {30, 0.5}, {30, 0.3}, {60, 0.5}, {3, 0.5}, {0, 0.5}, {20, 0.1}};
        for (double[] cs : cases) {
            double v0 = cs[0], wait = cs[1];
            Ball a = new Ball(new Pose(20, 20), POLLEN);
            long start = System.nanoTime();
            Thread.sleep((long) (wait * 1000));
            double T = (System.nanoTime() - start) / 1e9;
            double moving = Math.min(T, v0 / DECEL);
            double dist = v0 * moving - 0.5 * DECEL * moving * moving;
            double real = Math.max(0, v0 - DECEL * T);
            Ball b = new Ball(new Pose(20 + dist, 20), POLLEN);
            a.setPose(b);
            double got = a.getVelocity() == null ? Double.NaN : a.getVelocity().x();
            System.out.printf("  starts %4.1f in/s, seen again %.3f s later: real speed %5.2f, setPose says %5.2f%n", v0, T, real, got);
            c.check(Math.abs(got - real) < 0.3, String.format("speed correction v0=%.1f T=%.3f: got %.2f, real %.2f", v0, T, got, real));
        }
        c.finish();
    }

    /* =============================== 8. full getEndPoses =============================== */

    @Test
    public void s08_fullGetEndPoses() {
        Checks c = new Checks("8. Full getEndPoses with balls rolling in front of the camera");
        MatchInfo.alliance = GlobalConstants.BLUE_ALLIANCE;
        Pose fb = FACING_BACK;
        List<Scen> list = new ArrayList<>();
        list.add(new Scen("3 still balls", fb).ball(POLLEN, 72, 60, 0, 0).ball(BLUE_N, 60, 75, 0, 0).ball(POLLEN, 85, 90, 0, 0));
        list.add(new Scen("rolling +x, 20 in/s", fb).ball(POLLEN, 60, 70, 20, 0));
        list.add(new Scen("rolling into the blue hive, 30 in/s", fb).ball(POLLEN, 80, 70, 30, 0));
        list.add(new Scen("rolling away, 25 in/s", fb).ball(POLLEN, 72, 60, 0, 25));
        list.add(new Scen("rolling toward robot, leaves view", fb).ball(POLLEN, 72, 52, 0, -25));
        list.add(new Scen("fast 60 in/s, hits back wall later", fb).ball(POLLEN, 60, 60, 0, 60));
        list.add(new Scen("opponent nectar ignored", fb).ball(RED_N, 72, 70, 0, 0).ball(POLLEN, 80, 80, 0, 0));
        list.add(new Scen("blocked 0.05-0.15 s", fb).ball(POLLEN, 60, 70, 20, 0).block(0, 0.05, 0.15));
        list.add(new Scen("still balls, +/-2 px noise", fb).ball(POLLEN, 72, 60, 0, 0).ball(BLUE_N, 60, 75, 0, 0).ball(POLLEN, 85, 100, 0, 0).jitter(2));
        list.add(new Scen("rolling +x 20 in/s, +/-2 px noise", fb).ball(POLLEN, 60, 70, 20, 0).jitter(2));
        list.add(new Scen("far still ball (y=115), +/-2 px noise", fb).ball(POLLEN, 72, 115, 0, 0).jitter(2));
        list.add(new Scen("two still pollen 3.5 in apart", fb).ball(POLLEN, 70, 70, 0, 0).ball(POLLEN, 73.5, 70, 0, 0));
        list.add(new Scen("two touching pollen, camera merges them", fb).ball(POLLEN, 70, 70, 0, 0).ball(POLLEN, 72.9, 70, 0, 0).merge());
        list.add(new Scen("two pollen passing 3.5 in apart", fb).ball(POLLEN, 66, 74, 20, 0).ball(POLLEN, 78, 77.5, -20, 0));
        list.add(new Scen("rolls out the side of the view", fb).ball(POLLEN, 100, 110, 25, 0));
        list.add(new Scen("robot facing +x, ball into red hive end", new Pose(10, 30, 0)).ball(POLLEN, 48, 35, 0, 25));
        list.add(new Scen("robot heading -135 deg", new Pose(120, 120, Math.toRadians(-135))).ball(POLLEN, 80, 85, -10, 5).ball(BLUE_N, 90, 75, 0, 0));
        list.add(new Scen("6 balls, some moving", fb).ball(POLLEN, 60, 60, 10, 5).ball(POLLEN, 85, 65, 0, 0).ball(BLUE_N, 70, 85, -8, 0)
                .ball(POLLEN, 55, 100, 0, 0).ball(BLUE_N, 90, 105, 0, -10).ball(POLLEN, 72, 120, 5, 0));
        list.add(new Scen("80 in/s at 30 fps", fb).ball(POLLEN, 58, 75, 80, 0));
        list.add(new Scen("80 in/s at 15 fps", fb).ball(POLLEN, 58, 75, 80, 0).fps(15));
        list.add(new Scen("150 in/s at 30 fps", fb).ball(POLLEN, 52, 110, 150, 0));
        list.add(new Scen("150 in/s at 60 fps", fb).ball(POLLEN, 52, 110, 150, 0).fps(60));
        list.add(new Scen("two pollen rolling together, merged", fb).ball(POLLEN, 62, 70, 15, 0).ball(POLLEN, 64.8, 70, 15, 0).merge());
        list.add(new Scen("WHAT-IF edge blobs ignored: toward robot", fb).ball(POLLEN, 72, 52, 0, -25).dropEdge());
        list.add(new Scen("WHAT-IF edge blobs ignored: out the side", fb).ball(POLLEN, 100, 110, 25, 0).dropEdge());
        list.add(new Scen("WHAT-IF no frame delay: toward robot", fb).ball(POLLEN, 72, 52, 0, -25).dropEdge().fps(1000));
        list.add(new Scen("WHAT-IF no frame delay: out the side", fb).ball(POLLEN, 100, 110, 25, 0).dropEdge().fps(1000));
        list.add(new Scen("WHAT-IF no frame delay: rolling +x 20", fb).ball(POLLEN, 60, 70, 20, 0).fps(1000));

        System.out.println("  (checked: no exceptions and one end pose per ball in the first frame. Errors are measurements.)");
        for (Scen s : list) {
            Run r = run(s, BLUE_N);
            if (r.error != null) {
                c.check(false, s.name + " threw " + r.error);
                System.out.println("  " + s.name + ": THREW " + r.error);
                continue;
            }
            System.out.printf("  %s  (%d reads in %.2f s, %d frames)%n", s.name, r.reads, r.seconds, r.frames);
            //balls that bounce near a corner may be dropped
            boolean countOk = r.ends.size() <= r.expected.size() && r.ends.size() >= r.expected.size() - r.mayDrop;
            c.check(countOk, s.name + ": got " + r.ends.size() + " end poses, expected " + r.expected.size()
                    + (r.mayDrop > 0 ? " (" + r.mayDrop + " may be dropped for hitting a corner)" : ""));
            if (!countOk)
                System.out.printf("    got %d end poses, expected %d%n", r.ends.size(), r.expected.size());
            for (double[] row : match(s, r)) {
                SimBall b = s.balls.get((int) row[0]);
                if (row[1] == -2) {
                    System.out.printf("    %-10s from %-13s v=(%4.0f,%4.0f)  dropped: its real path bounces %.2f in from a corner%n",
                            color(b.id), p2(b.startX, b.startY), b.startVx, b.startVy, b.cornerDist);
                    continue;
                }
                if (row[1] < 0) {
                    System.out.printf("    %s from %s: no end pose left to match%n", color(b.id), p2(b.startX, b.startY));
                    continue;
                }
                Pose end = r.ends.get((int) row[1]);
                System.out.printf("    %-10s from %-13s v=(%4.0f,%4.0f)  real end %-14s predicted %-14s error %5.2f  bounces %d%s%n",
                        color(b.id), p2(b.startX, b.startY), b.startVx, b.startVy, p2(b.x, b.y),
                        p2(end.x(), end.y()), row[2], b.bounces, row[2] > 3 ? "   <-- off by more than 3 in" : "");
            }
        }
        c.finish();
    }

    /* =============================== 9. calling getEndPoses again and again =============================== */

    @Test
    public void s09_repeatedCalls() {
        Checks c = new Checks("9. Calling getEndPoses again and again");
        MatchInfo.alliance = GlobalConstants.BLUE_ALLIANCE;

        // a) the same still scene, many calls in a row
        Scen still = new Scen("3 still balls", FACING_BACK).ball(POLLEN, 72, 60, 0, 0).ball(BLUE_N, 60, 75, 0, 0).ball(POLLEN, 85, 90, 0, 0);
        int calls = 15;
        List<Pose> first = null;
        double maxDiff = 0, maxErr = 0;
        int ballsBefore = new Ball(new Pose(0, 0), POLLEN).getBallID();
        for (int i = 0; i < calls; i++) {
            Run r = run(still, BLUE_N);
            c.check(r.error == null, "call " + (i + 1) + " threw " + r.error);
            if (r.error != null) continue;
            c.check(r.ends.size() == 3, "call " + (i + 1) + " gave " + r.ends.size() + " end poses, expected 3");
            if (first == null) {
                first = r.ends;
            } else if (r.ends.size() == first.size()) {
                for (int k = 0; k < first.size(); k++) maxDiff = Math.max(maxDiff, first.get(k).distance(r.ends.get(k)));
            }
            for (double[] row : match(still, r)) maxErr = Math.max(maxErr, Double.isNaN(row[2]) ? 1e9 : row[2]);
        }
        int ballsPerCall = (new Ball(new Pose(0, 0), POLLEN).getBallID() - ballsBefore - 1) / calls;
        System.out.printf("  a) 3 still balls, %d calls in a row: biggest difference from the first call %.1e in, worst error vs the real spot %.2f in%n",
                calls, maxDiff, maxErr);
        System.out.printf("     about %d Ball objects made per call; nothing else is kept between calls%n", ballsPerCall);
        c.check(maxDiff < 1e-9, String.format("same still scene gave different end poses on different calls (up to %.4f in apart)", maxDiff));
        c.check(maxErr < 0.5, String.format("still balls predicted up to %.2f in from where they are", maxErr));

        // b) rolling balls: each call starts at a random time in the roll and a random point in the camera's frame cycle
        System.out.println("  b) Rolling balls, 12 calls each, started at a random time in the roll and a random point in the camera's frame cycle:");
        Object[][] rolling = {
                {"rolling +x, 20 in/s", 60, 70, 20, 0},
                {"rolling away, 25 in/s", 72, 60, 0, 25},
                {"slow diagonal, 10 in/s", 65, 70, 7, 7},
        };
        Random rnd = new Random(9);
        for (Object[] rc : rolling) {
            String name = (String) rc[0];
            List<Pose> ends = new ArrayList<>();
            Stats err = new Stats();
            for (int i = 0; i < 12; i++) {
                Scen s = new Scen(name, FACING_BACK)
                        .ball(POLLEN, ((Number) rc[1]).doubleValue(), ((Number) rc[2]).doubleValue(), ((Number) rc[3]).doubleValue(), ((Number) rc[4]).doubleValue())
                        .startAt(0.05 + rnd.nextDouble() * 0.25, rnd.nextDouble() * FRAME_S);
                Run r = run(s, BLUE_N);
                c.check(r.error == null, name + " threw " + r.error);
                if (r.error != null || r.ends.size() != 1) continue;
                ends.add(r.ends.get(0));
                List<double[]> rows = match(s, r);
                if (!rows.isEmpty()) err.add(rows.get(0)[2]);
            }
            double ax = 0, ay = 0;
            for (Pose p : ends) { ax += p.x() / ends.size(); ay += p.y() / ends.size(); }
            double spread = 0;
            for (Pose p : ends) spread = Math.max(spread, Math.hypot(p.x() - ax, p.y() - ay));
            System.out.printf("     %-24s spread between calls %.2f in   error vs real end: mean %.2f  max %.2f%n", name, spread, err.mean(), err.max);
        }

        // c) the robot moves while getEndPoses runs, but getEndPoses only has the pose from the start of the call
        System.out.println("  c) Robot moving during the call (3 still balls; getEndPoses is given the pose from when it started):");
        Object[][] motions = {
                {"robot stopped", 0, 0, 0},
                {"creeping forward 5 in/s", 0, 5, 0},
                {"driving forward 20 in/s", 0, 20, 0},
                {"driving sideways 20 in/s", 20, 0, 0},
                {"turning 30 deg/s", 0, 0, 30},
                {"turning 90 deg/s", 0, 0, 90},
        };
        for (Object[] m : motions) {
            Scen s = new Scen((String) m[0], FACING_BACK).ball(POLLEN, 72, 60, 0, 0).ball(BLUE_N, 60, 75, 0, 0).ball(POLLEN, 85, 90, 0, 0)
                    .robotMoving(((Number) m[1]).doubleValue(), ((Number) m[2]).doubleValue(), Math.toRadians(((Number) m[3]).doubleValue()));
            Run r = run(s, BLUE_N);
            if (r.error != null) {
                c.check(false, s.name + " threw " + r.error);
                System.out.println("     " + s.name + ": THREW " + r.error);
                continue;
            }
            Stats err = new Stats();
            for (double[] row : match(s, r)) if (!Double.isNaN(row[2])) err.add(row[2]);
            System.out.printf("     %-26s error vs where the balls really are: mean %5.1f  max %5.1f in   (%d end poses)%n",
                    s.name, err.mean(), err.max, r.ends.size());
        }
        c.finish();
    }

    /* =============================== 10. reading balls while driving =============================== */

    @Test
    public void s10_readingWhileDriving() {
        Checks c = new Checks("10. getVisibleBalls while the robot drives (checking end poses on the way)");
        int alliance = GlobalConstants.BLUE_ALLIANCE;
        double frameAge = FRAME_S;
        System.out.printf("  The newest camera frame can be up to %.0f ms old when it's read (more if the HuskyLens takes time to process; not measured).%n", frameAge * 1000);

        // straight at a still pollen
        double driveSpeed = 30;
        System.out.printf("  Driving straight at a still pollen at %.0f in/s:%n", driveSpeed);
        System.out.printf("    %-24s %-30s %s%n", "distance from robot center", "error, frame just taken", String.format("error, frame %.0f ms old", frameAge * 1000));
        Pose now = new Pose(72, 30, Math.PI / 2);
        for (double d : new double[]{90, 70, 50, 40, 30, 25, 23, 22, 21, 20, 18}) {
            double bx = 72, by = 30 + d;
            String[] cells = new String[2];
            for (int a = 0; a < 2; a++) {
                double age = a == 0 ? 0 : frameAge;
                Pose taken = new Pose(now.x(), now.y() - driveSpeed * age, now.heading());
                cells[a] = readOnce(c, alliance, now, taken, bx, by, age == 0);
            }
            System.out.printf("    %-24s %-30s %s%n", String.format("%.0f in", d), cells[0], cells[1]);
        }

        // turning
        double turn = 90;
        System.out.printf("  Turning at %.0f deg/s, still pollen straight ahead (frame %.0f ms old):%n", turn, frameAge * 1000);
        for (double d : new double[]{90, 60, 40, 25}) {
            double bx = 72, by = 30 + d;
            Pose taken = new Pose(now.x(), now.y(), now.heading() - Math.toRadians(turn) * frameAge);
            System.out.printf("    %-24s %s%n", String.format("%.0f in", d), readOnce(c, alliance, now, taken, bx, by, false));
        }

        // how close a ball can get before the camera loses it
        double fullyVisible = Double.NaN, centerVisible = Double.NaN;
        for (double d = 40; d >= 5; d -= 0.1) {
            int[] box = boxFor(POLLEN, projectRel(d, 0, R_POLLEN), 0, 0);
            if (box == null || box[2] < TOP_CUT) break;
            if (box[5] == 0) fullyVisible = d;
            if (projectRel(d, 0, R_POLLEN)[1] <= IMG_H) centerVisible = d;
        }
        System.out.printf("  A pollen straight ahead is fully in view down to %.1f in from the robot center, and half in view down to %.1f in.%n",
                fullyVisible, centerVisible);
        System.out.println("  Closer than that the camera can't see it, so keep the last place it was seen for the final approach.");
        c.finish();
    }

    /** Reads one frame taken from pose `taken` with getVisibleBalls given pose `now`. */
    private static String readOnce(Checks c, int alliance, Pose now, Pose taken, double bx, double by, boolean checkIt) {
        double[] p = projectField(taken, bx, by, R_POLLEN);
        int[] box = p == null ? null : boxFor(POLLEN, p, 0, 0);
        if (box == null || box[2] < TOP_CUT) return "not seen";
        List<Ball> got = visible(alliance, now, toBlock(box));
        if (got.isEmpty()) return "not seen (filtered out)";
        double e = Math.hypot(got.get(0).getPose().x() - bx, got.get(0).getPose().y() - by);
        if (checkIt && box[5] == 0) c.check(e < 0.6, String.format("fresh frame, ball %.1f in ahead read %.2f in off", by - now.y(), e));
        return String.format("%.2f in%s", e, box[5] == 1 ? " (cut off at the edge)" : "");
    }

    @SafeVarargs
    private static <T> List<T> listOf(T... items) {
        List<T> out = new ArrayList<>();
        Collections.addAll(out, items);
        return Collections.unmodifiableList(out);
    }
}
