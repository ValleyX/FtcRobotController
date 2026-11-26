package org.firstinspires.ftc.team2844.recognition;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team2844.subsystems.HuskySubsystem;
import org.firstinspires.ftc.team2844global.Ball;
import org.firstinspires.ftc.team2844global.GlobalConstants;
import org.firstinspires.ftc.team2844global.collision.CollisionBox;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.function.Supplier;

import static org.junit.Assert.fail;

/**
 * Everything RecognitionTests needs that isn't a test itself:
 * <ul>
 *     <li>a fake HuskyLens that hands HuskySubsystem real 16-byte blocks,</li>
 *     <li>a perfect pinhole camera that turns a ball on the field into the block the HuskyLens would report,</li>
 *     <li>an independent ray-cast to check HuskySubsystem's camera math against,</li>
 *     <li>a small-time-step physics sim to check Ball's predictions against,</li>
 *     <li>scenarios that play a whole getEndPoses call in real time with balls rolling in front of the camera.</li>
 * </ul>
 * All the camera and ball settings are read from the real HuskySubsystem and Ball,
 * so changing a constant there changes the tests too.
 */
final class RecognitionSim {
    private RecognitionSim() { }

    /* ---------- settings, read from the robot code in setup() ---------- */
    static double TOP_CUT, IMG_W, IMG_H, TILT, CAM_H, VFOV, HFOV, FWD_OFF, RIGHT_OFF, MAX_MATCH;
    static double DECEL, EFF, CORNER_MARGIN, R_POLLEN, R_NECTAR;
    /** In a getEndPoses scenario the code only knows the ball's position and speed roughly, so a ball whose real path
     * passes within CORNER_MARGIN + this many inches of a corner is allowed to be dropped. */
    static final double SCENARIO_CORNER_SLACK = 1.0;
    static final double FIELD = GlobalConstants.FIELD_SIZE;
    static final int POLLEN = GlobalConstants.POLLEN;
    static final int BLUE_N = GlobalConstants.BLUE_NECTAR;
    static final int RED_N = GlobalConstants.RED_NECTAR;

    /* ---------- camera behavior: GUESSES, not measured on our HuskyLens ---------- */
    /** Seconds between camera frames (30 fps). */
    static double FRAME_S = 1.0 / 30.0;
    /** How long one huskyLens.blocks() read takes during a getEndPoses scenario, in ms. */
    static double READ_MS = 5.0;

    static FakeHusky cam;
    static HuskySubsystem husky;
    /** {minX, minY, maxX, maxY} of every box in ALL_COLLISIONS / COLLISIONS. */
    static double[][] allBoxes, obstacleBoxes;
    private static boolean ready = false;

    /** A HuskyLens that returns whatever blocks the test gives it instead of talking to a real camera. */
    static class FakeHusky extends HuskyLens {
        Supplier<HuskyLens.Block[]> source;
        int reads;
        double readMs;

        /** Never runs: setup() makes this with allocateInstance, because the real constructor needs an I2C device. */
        FakeHusky() { super(null); }

        @Override
        public HuskyLens.Block[] blocks() {
            reads++;
            if (readMs > 0) {
                try { Thread.sleep((long) readMs); } catch (InterruptedException ignored) { }
            }
            return source.get();
        }

        @Override
        public void selectAlgorithm(HuskyLens.Algorithm algorithm) { }
    }

    /** A HardwareMap that hands out the fake HuskyLens, so HuskySubsystem's real constructor can run. */
    static class FakeHardwareMap extends HardwareMap {
        /** Never runs: setup() makes this with allocateInstance, because the real constructor needs an Android Context. */
        FakeHardwareMap() { super(null, null); }

        @Override
        public <T> T get(Class<? extends T> classOrInterface, String deviceName) {
            return classOrInterface.cast(cam);
        }
    }

    /* =============================== setup =============================== */

    /** Makes an object without running its constructor. */
    private static Object allocate(Class<?> c) throws Exception {
        Class<?> unsafeClass = Class.forName("sun.misc.Unsafe");
        Field theUnsafe = unsafeClass.getDeclaredField("theUnsafe");
        theUnsafe.setAccessible(true);
        Method allocateInstance = unsafeClass.getMethod("allocateInstance", Class.class);
        return allocateInstance.invoke(theUnsafe.get(null), c);
    }

    private static double setting(Class<?> c, Object instance, String name) throws Exception {
        Field f;
        try {
            f = c.getDeclaredField(name);
        } catch (NoSuchFieldException e) {
            throw new IllegalStateException("The tests read " + name + " from " + c.getSimpleName()
                    + ", but it isn't there anymore. Update RecognitionSim.setup().", e);
        }
        f.setAccessible(true);
        return ((Number) f.get(Modifier.isStatic(f.getModifiers()) ? null : instance)).doubleValue();
    }

    static synchronized void setup() throws Exception {
        if (ready) return;
        cam = (FakeHusky) allocate(FakeHusky.class);
        cam.source = () -> new HuskyLens.Block[0];
        husky = new HuskySubsystem((HardwareMap) allocate(FakeHardwareMap.class));

        TOP_CUT = setting(HuskySubsystem.class, husky, "TOP_CUTTOFF");
        IMG_W = setting(HuskySubsystem.class, husky, "CAMERA_WIDTH");
        IMG_H = setting(HuskySubsystem.class, husky, "CAMERA_HEIGHT");
        TILT = setting(HuskySubsystem.class, husky, "CAMERA_TILT");
        CAM_H = setting(HuskySubsystem.class, husky, "HEIGHT_ON_ROBOT");
        VFOV = setting(HuskySubsystem.class, husky, "vFOV");
        HFOV = setting(HuskySubsystem.class, husky, "hFOV");
        FWD_OFF = setting(HuskySubsystem.class, husky, "FORWARD_OFFSET");
        RIGHT_OFF = setting(HuskySubsystem.class, husky, "RIGHT_OFFSET");
        MAX_MATCH = setting(HuskySubsystem.class, husky, "MAX_MATCH_DISTANCE");
        DECEL = setting(Ball.class, null, "ROLLING_DECEL");
        EFF = setting(Ball.class, null, "efficiency");
        CORNER_MARGIN = setting(Ball.class, null, "CORNER_MARGIN");
        R_POLLEN = new Ball(new Pose(0, 0), POLLEN).getBallRadius();
        R_NECTAR = new Ball(new Pose(0, 0), BLUE_N).getBallRadius();

        allBoxes = toBoxes(GlobalConstants.ALL_COLLISIONS);
        obstacleBoxes = toBoxes(GlobalConstants.COLLISIONS);
        ready = true;
    }

    static void printSettings() {
        System.out.printf("Settings read from the robot code: TOP_CUTTOFF=%d image=%dx%d tilt=%.1f height=%.1f vFOV=%.1f hFOV=%.1f%n",
                (int) TOP_CUT, (int) IMG_W, (int) IMG_H, TILT, CAM_H, VFOV, HFOV);
        System.out.printf("  FORWARD_OFFSET=%.1f RIGHT_OFFSET=%.1f MAX_MATCH_DISTANCE=%.1f ROLLING_DECEL=%.1f efficiency=%.2f CORNER_MARGIN=%.1f%n",
                FWD_OFF, RIGHT_OFF, MAX_MATCH, DECEL, EFF, CORNER_MARGIN);
        System.out.printf("  Camera guesses (RecognitionSim): %.0f fps, %.0f ms per read%n", 1 / FRAME_S, READ_MS);
    }

    static double[][] toBoxes(CollisionBox[] cs) {
        double[][] out = new double[cs.length][];
        for (int i = 0; i < cs.length; i++) {
            double minX = 1e9, minY = 1e9, maxX = -1e9, maxY = -1e9;
            for (Pose p : cs[i].getCorners()) {
                minX = Math.min(minX, p.x()); minY = Math.min(minY, p.y());
                maxX = Math.max(maxX, p.x()); maxY = Math.max(maxY, p.y());
            }
            out[i] = new double[]{minX, minY, maxX, maxY};
        }
        return out;
    }

    /* =============================== pass/fail =============================== */

    /** Collects the checks for one test, prints a summary, and fails the test if any check failed. */
    static final class Checks {
        private final List<String> failures = new ArrayList<>();
        private int passed, failed;

        Checks(String title) {
            System.out.println();
            System.out.println("==== " + title + " ====");
        }

        void check(boolean ok, String what) {
            if (ok) {
                passed++;
            } else {
                failed++;
                if (failures.size() < 200) failures.add(what);
            }
        }

        void finish() {
            System.out.printf("  -> %d checks passed, %d failed%n", passed, failed);
            if (failed == 0) return;
            StringBuilder sb = new StringBuilder(failed + " of " + (passed + failed) + " checks failed:");
            for (int i = 0; i < Math.min(25, failures.size()); i++) sb.append("\n  ").append(failures.get(i));
            if (failed > 25) sb.append("\n  ... ").append(failed - 25).append(" more");
            System.out.println(sb);
            fail(sb.toString());
        }
    }

    /* =============================== camera =============================== */

    static double radius(int id) { return id == POLLEN ? R_POLLEN : R_NECTAR; }

    static String color(int id) {
        return id == POLLEN ? "pollen" : id == BLUE_N ? "blueNectar" : id == RED_N ? "redNectar" : "id" + id;
    }

    private static void put(byte[] b, int at, int v) {
        b[at] = (byte) (v & 0xFF);
        b[at + 1] = (byte) ((v >> 8) & 0xFF);
    }

    /** The block the HuskyLens driver would make from a real 16-byte message. */
    static HuskyLens.Block block(int id, int x, int y, int w, int h) {
        byte[] b = new byte[16];
        put(b, 5, x); put(b, 7, y); put(b, 9, w); put(b, 11, h); put(b, 13, id);
        return cam.new Block(b);
    }

    static double kx() { return (IMG_W / 2) / Math.tan(Math.toRadians(HFOV) / 2); }
    static double ky() { return (IMG_H / 2) / Math.tan(Math.toRadians(VFOV) / 2); }

    /** Field point to robot-relative {forward, right}. */
    static double[] robotRel(Pose robot, double bx, double by) {
        double h = robot.heading(), dx = bx - robot.x(), dy = by - robot.y();
        return new double[]{dx * Math.cos(h) + dy * Math.sin(h), dx * Math.sin(h) - dy * Math.cos(h)};
    }

    /** Robot-relative {forward, right} to a field point. */
    static double[] toField(Pose robot, double fwd, double right) {
        double h = robot.heading();
        return new double[]{robot.x() + fwd * Math.cos(h) + right * Math.sin(h), robot.y() + fwd * Math.sin(h) - right * Math.cos(h)};
    }

    /** True pinhole projection: {px, py, radiusPxX, radiusPxY} for a ball at robot-relative fwd/right, or null if behind the camera. */
    static double[] projectRel(double fwd, double right, double r) {
        double f = fwd - FWD_OFF, s = right - RIGHT_OFF, drop = CAM_H - r, t = Math.toRadians(TILT);
        double depth = f * Math.cos(t) + drop * Math.sin(t);
        double down = -f * Math.sin(t) + drop * Math.cos(t);
        if (depth < 1.0) return null;
        return new double[]{IMG_W / 2 + kx() * s / depth, IMG_H / 2 + ky() * down / depth, kx() * r / depth, ky() * r / depth};
    }

    static double[] projectField(Pose robot, double bx, double by, double r) {
        double[] rel = robotRel(robot, bx, by);
        return projectRel(rel[0], rel[1], r);
    }

    /** The box the camera would report, cut off at the image edges: {id, x, y, w, h, clipped 0/1}, or null if off screen. */
    static int[] boxFor(int id, double[] p, double jx, double jy) {
        double cx = p[0] + jx, cy = p[1] + jy;
        double l = Math.max(0, cx - p[2]), r = Math.min(IMG_W, cx + p[2]);
        double t = Math.max(0, cy - p[3]), b = Math.min(IMG_H, cy + p[3]);
        if (r - l < 1 || b - t < 1) return null;
        boolean clipped = l > cx - p[2] || r < cx + p[2] || t > cy - p[3] || b < cy + p[3];
        return new int[]{id, (int) Math.round((l + r) / 2), (int) Math.round((t + b) / 2),
                (int) Math.round(r - l), (int) Math.round(b - t), clipped ? 1 : 0};
    }

    static HuskyLens.Block toBlock(int[] bx) { return block(bx[0], bx[1], bx[2], bx[3], bx[4]); }

    static HuskyLens.Block[] toBlocks(List<int[]> boxes) {
        HuskyLens.Block[] out = new HuskyLens.Block[boxes.size()];
        for (int i = 0; i < out.length; i++) out[i] = toBlock(boxes.get(i));
        return out;
    }

    /** Independent inverse: rotate the pixel's ray by the tilt and hit the plane at ball-center height. Robot-relative {fwd, right}. */
    static double[] raycast(int px, int py, double r) {
        double a = (px - IMG_W / 2) / (IMG_W / 2) * Math.tan(Math.toRadians(HFOV) / 2);
        double b = (py - IMG_H / 2) / (IMG_H / 2) * Math.tan(Math.toRadians(VFOV) / 2);
        double t = Math.toRadians(TILT);
        double rayF = Math.cos(t) - b * Math.sin(t), rayUp = -Math.sin(t) - b * Math.cos(t);
        if (rayUp >= 0) return null;
        double s = (CAM_H - r) / -rayUp;
        return new double[]{s * rayF + FWD_OFF, s * a + RIGHT_OFF};
    }

    /** What HuskySubsystem's math says, robot-relative {fwd, right}. */
    static double[] codeRel(HuskyLens.Block b) {
        double f = husky.getForwardInches(b);
        return new double[]{f + FWD_OFF, husky.getHorizontalInches(b, f) + RIGHT_OFF};
    }

    /** Runs getVisibleBalls on one camera frame made of these blocks. */
    static List<Ball> visible(int alliance, Pose robot, HuskyLens.Block... blocks) {
        cam.source = () -> blocks;
        return husky.getVisibleBalls(alliance, robot);
    }

    /* =============================== field =============================== */

    static double distToBox(double x, double y, double[] b) {
        double cx = Math.max(b[0], Math.min(x, b[2])), cy = Math.max(b[1], Math.min(y, b[3]));
        return Math.hypot(x - cx, y - cy);
    }

    static boolean clearSpot(double x, double y, double r, double margin) {
        if (x < r + margin || x > FIELD - r - margin || y < r + margin || y > FIELD - r - margin) return false;
        for (double[] b : allBoxes) if (distToBox(x, y, b) < r + margin) return false;
        return true;
    }

    static boolean insideObstacle(double x, double y, double r) {
        for (double[] b : obstacleBoxes) if (distToBox(x, y, b) < r - 0.05) return true;
        return false;
    }

    static boolean outsideField(double x, double y, double r) {
        return x < r - 0.05 || x > FIELD - r + 0.05 || y < r - 0.05 || y > FIELD - r + 0.05;
    }

    static String p2(double x, double y) { return String.format("(%.1f, %.1f)", x, y); }

    static class Stats {
        int n;
        double sum, max;
        final List<Double> all = new ArrayList<>();

        void add(double v) { n++; sum += v; max = Math.max(max, v); all.add(v); }
        double mean() { return n == 0 ? 0 : sum / n; }

        double pct(double p) {
            if (all.isEmpty()) return 0;
            List<Double> s = new ArrayList<>(all);
            Collections.sort(s);
            return s.get(Math.min(s.size() - 1, (int) Math.floor(p * s.size())));
        }

        String line() { return String.format("n=%-6d mean=%6.2f  95%%=%6.2f  max=%6.2f", n, mean(), pct(0.95), max); }
    }

    /* =============================== reference physics =============================== */

    /** Small-time-step simulation: constant slowdown DECEL, bounces off every ALL_COLLISIONS box with EFF.
     * Runs to a stop in the constructor; x and y are where it stops. With record on, at(t) gives where it was t seconds in. */
    static class SimBall {
        final int id;
        final double r;
        double x, y, vx, vy;
        int bounces;
        int n;
        /** Closest any bounce point comes to a free box corner (one not on or outside the field edge), in inches. */
        double cornerDist = Double.POSITIVE_INFINITY;
        final double startX, startY, startVx, startVy;
        double[] tx, ty;

        SimBall(int id, double x, double y, double vx, double vy, boolean record) {
            this.id = id; r = radius(id); this.x = x; this.y = y; this.vx = vx; this.vy = vy;
            startX = x; startY = y; startVx = vx; startVy = vy;
            if (record) { tx = new double[40001]; ty = new double[40001]; tx[0] = x; ty[0] = y; }
            int ms = 0;
            while (ms < 40000 && (this.vx != 0 || this.vy != 0)) {
                for (int k = 0; k < 5; k++) step(0.0002);
                ms++;
                if (record) { tx[ms] = this.x; ty[ms] = this.y; }
            }
            n = ms;
        }

        void step(double dt) {
            double s = Math.hypot(vx, vy);
            if (s == 0) return;
            double ns = Math.max(0, s - DECEL * dt);
            double used = ns > 0 ? dt : s / DECEL;
            double dist = s * used - 0.5 * DECEL * used * used;
            double ux = vx / s, uy = vy / s;
            x += ux * dist; y += uy * dist;
            vx = ux * ns; vy = uy * ns;
            for (double[] b : allBoxes) {
                double cx = Math.max(b[0], Math.min(x, b[2])), cy = Math.max(b[1], Math.min(y, b[3]));
                double dx = x - cx, dy = y - cy, d = Math.hypot(dx, dy);
                if (d < r && d > 1e-12) {
                    double nx = dx / d, ny = dy / d, vn = vx * nx + vy * ny;
                    if (vn < 0) {
                        vx -= (1 + EFF) * vn * nx; vy -= (1 + EFF) * vn * ny; bounces++;
                        for (double[] c : new double[][]{{b[0], b[1]}, {b[2], b[1]}, {b[2], b[3]}, {b[0], b[3]}}) {
                            if (c[0] <= 0 || c[0] >= FIELD || c[1] <= 0 || c[1] >= FIELD) continue;
                            cornerDist = Math.min(cornerDist, Math.hypot(cx - c[0], cy - c[1]));
                        }
                    }
                    x = cx + nx * r; y = cy + ny * r;
                }
            }
        }

        double[] at(double t) {
            int i = (int) Math.round(t * 1000);
            if (i > n) i = n;
            if (i < 0) i = 0;
            return new double[]{tx[i], ty[i]};
        }
    }

    /* =============================== getEndPoses scenarios =============================== */

    /** Balls rolling in front of the robot while getEndPoses runs, played back in real time through the fake camera. */
    static class Scen {
        final String name;
        /** Robot pose at the start of the call. This is the pose getEndPoses is given. */
        final Pose robot;
        final List<SimBall> balls = new ArrayList<>();
        double jitter = 0;
        boolean merge = false, dropEdge = false;
        final Map<Integer, double[]> blocked = new HashMap<>();
        double frameS = FRAME_S;
        /** Robot motion during the call, field in/s and rad/s. */
        double robotVx, robotVy, robotTurn;
        /** Seconds into the balls' roll when getEndPoses is called. */
        double callStart = 0;
        /** How old the newest camera frame is when getEndPoses is called, in seconds (0 to frameS). */
        double phase = 0;

        Scen(String name, Pose robot) { this.name = name; this.robot = robot; }

        Scen ball(int id, double x, double y, double vx, double vy) { balls.add(new SimBall(id, x, y, vx, vy, true)); return this; }
        /** Random +/- pixel noise on every block. */
        Scen jitter(double px) { jitter = px; return this; }
        /** Same-color blobs that touch come back as one block, like the HuskyLens probably does. */
        Scen merge() { merge = true; return this; }
        /** Ball number `ball` can't be seen from `from` to `to` seconds. */
        Scen block(int ball, double from, double to) { blocked.put(ball, new double[]{from, to}); return this; }
        Scen fps(double fps) { frameS = 1.0 / fps; return this; }
        /** Leaves out blobs cut off at the image edge (a what-if, the code doesn't do this). */
        Scen dropEdge() { dropEdge = true; return this; }
        Scen robotMoving(double vx, double vy, double turnRadPerS) { robotVx = vx; robotVy = vy; robotTurn = turnRadPerS; return this; }
        Scen startAt(double callStart, double phase) { this.callStart = callStart; this.phase = phase; return this; }

        /** Where the robot really is t seconds after the call started. */
        Pose robotAt(double t) {
            return new Pose(robot.x() + robotVx * t, robot.y() + robotVy * t, robot.heading() + robotTurn * t);
        }
    }

    /** {id, x, y, w, h, clipped, ballIndex} for camera frame number frameIdx. ballIndex -2 means two balls merged. */
    static List<int[]> frameBoxes(Scen s, int frameIdx) {
        //seconds after the call started that this frame was taken (frame 0 is the newest one when the call starts)
        double t = frameIdx * s.frameS - s.phase;
        Random rnd = new Random(frameIdx * 7919L + s.name.hashCode());
        Pose robot = s.robotAt(t);
        List<int[]> raw = new ArrayList<>();
        for (int i = 0; i < s.balls.size(); i++) {
            double jx = s.jitter * (rnd.nextDouble() * 2 - 1), jy = s.jitter * (rnd.nextDouble() * 2 - 1);
            double[] bl = s.blocked.get(i);
            if (bl != null && t >= bl[0] && t < bl[1]) continue;
            SimBall b = s.balls.get(i);
            double[] pos = b.at(s.callStart + t);
            double[] p = projectField(robot, pos[0], pos[1], b.r);
            if (p == null) continue;
            int[] bx = boxFor(b.id, p, jx, jy);
            if (bx == null) continue;
            if (s.dropEdge && bx[5] == 1) continue;
            raw.add(new int[]{bx[0], bx[1], bx[2], bx[3], bx[4], bx[5], i});
        }
        if (s.merge) {
            boolean changed = true;
            while (changed) {
                changed = false;
                outer:
                for (int i = 0; i < raw.size(); i++)
                    for (int j = i + 1; j < raw.size(); j++) {
                        int[] a = raw.get(i), b = raw.get(j);
                        if (a[0] != b[0]) continue;
                        //touching or within 2 px counts as one blob
                        if (Math.abs(a[1] - b[1]) * 2 <= a[3] + b[3] + 4 && Math.abs(a[2] - b[2]) * 2 <= a[4] + b[4] + 4) {
                            int l = Math.min(a[1] - a[3] / 2, b[1] - b[3] / 2), r = Math.max(a[1] + a[3] / 2, b[1] + b[3] / 2);
                            int tp = Math.min(a[2] - a[4] / 2, b[2] - b[4] / 2), bt = Math.max(a[2] + a[4] / 2, b[2] + b[4] / 2);
                            raw.set(i, new int[]{a[0], (l + r) / 2, (tp + bt) / 2, r - l, bt - tp, 0, -2});
                            raw.remove(j);
                            changed = true;
                            break outer;
                        }
                    }
            }
        }
        Collections.shuffle(raw, rnd);
        return raw;
    }

    /** One getEndPoses call: what it returned and what it should have returned. */
    static class Run {
        List<Pose> ends;
        /** Blocks in the first frame that pass the filters: one end pose each is expected. */
        List<int[]> expected;
        /** How many of the expected balls come close enough to a corner that the code may drop them. */
        int mayDrop;
        int reads, frames;
        double seconds;
        Throwable error;
    }

    /** Calls the real getEndPoses with the fake camera playing the scenario in real time. */
    static Run run(Scen s, int nectarId) {
        Run r = new Run();
        r.expected = new ArrayList<>();
        for (int[] bx : frameBoxes(s, 0)) {
            if (bx[2] < TOP_CUT || (bx[0] != POLLEN && bx[0] != nectarId)) continue;
            double[] rel = codeRel(toBlock(bx));
            double[] f = toField(s.robot, rel[0], rel[1]);
            if (f[0] < 0 || f[0] > FIELD || f[1] < 0 || f[1] > FIELD) continue;
            r.expected.add(bx);
            if (bx[6] >= 0 && mayBeDropped(s.balls.get(bx[6]))) r.mayDrop++;
        }

        long start = System.nanoTime();
        Map<Integer, HuskyLens.Block[]> cache = new HashMap<>();
        cam.reads = 0;
        cam.readMs = READ_MS;
        cam.source = () -> {
            double t = (System.nanoTime() - start) / 1e9;
            int f = (int) Math.floor((t + s.phase) / s.frameS);
            return cache.computeIfAbsent(f, k -> toBlocks(frameBoxes(s, k)));
        };
        try {
            r.ends = husky.getEndPoses(s.robot);
        } catch (Throwable ex) {
            r.error = ex;
        } finally {
            cam.readMs = 0;
        }
        r.seconds = (System.nanoTime() - start) / 1e9;
        r.reads = cam.reads;
        r.frames = cache.size();
        return r;
    }

    /** True if the ball's real path bounces close enough to a corner that getEndPoses may leave it out. */
    static boolean mayBeDropped(SimBall b) {
        return b.cornerDist < CORNER_MARGIN + SCENARIO_CORNER_SLACK;
    }

    /** Pairs each expected ball with the closest unused end pose.
     * Rows are {ball index, end pose index (-1 if none was left, -2 if dropped for hitting a corner), error in inches}. */
    static List<double[]> match(Scen s, Run r) {
        List<double[]> rows = new ArrayList<>();
        boolean[] used = new boolean[r.ends.size()];
        //fewer end poses than balls: assume the ones near a corner were dropped
        boolean someDropped = r.ends.size() < r.expected.size();
        for (int[] bx : r.expected) {
            if (someDropped && bx[6] >= 0 && mayBeDropped(s.balls.get(bx[6]))) {
                rows.add(new double[]{bx[6], -2, Double.NaN});
                continue;
            }
            List<Integer> idx = new ArrayList<>();
            if (bx[6] >= 0) idx.add(bx[6]);
            else for (int i = 0; i < s.balls.size(); i++) if (s.balls.get(i).id == bx[0]) idx.add(i);
            for (int bi : idx) {
                SimBall b = s.balls.get(bi);
                int best = -1;
                double bestE = 1e9;
                for (int j = 0; j < r.ends.size(); j++) {
                    if (used[j]) continue;
                    double e = Math.hypot(r.ends.get(j).x() - b.x, r.ends.get(j).y() - b.y);
                    if (e < bestE) { bestE = e; best = j; }
                }
                rows.add(new double[]{bi, best, best < 0 ? Double.NaN : bestE});
                if (best < 0) break;
                if (bx[6] >= 0) used[best] = true;
            }
            //a merged blob is one end pose for two balls
            if (bx[6] < 0) {
                for (int j = 0; j < r.ends.size(); j++) if (!used[j]) { used[j] = true; break; }
            }
        }
        return rows;
    }
}
