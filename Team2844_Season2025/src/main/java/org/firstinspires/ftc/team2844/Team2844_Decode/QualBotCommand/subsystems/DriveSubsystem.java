package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.subsystems;

import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vcs.valleylib.core.time.RobotClock;
import com.vcs.valleylib.ftc.pedro.PedroSubsystem;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.PedroConstants;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.RobotConstants;

/**
 * The drivetrain, backed by a Pedro Pathing {@link com.pedropathing.follower.Follower}.
 *
 * <p>The follower does double duty: it follows paths in auto, and it provides
 * field-centric teleop drive plus the pose estimate the rest of the robot reads
 * its heading from. {@code PedroSubsystem.periodic()} calls {@code follower.update()}
 * once per scheduler loop, so nothing else needs to.
 *
 * <p>Pedro 3 replaced the old teleop-drive entry points with a single
 * {@code manual(DrivePowers)} that is robot-centric, plus {@link ManualDrive}
 * helpers that rotate driver input into the field frame. Turning in place is no
 * longer a dedicated call either: it is a {@code hold} of the current position
 * with a new heading.
 */
public class DriveSubsystem extends PedroSubsystem {

    /**
     * How long the follower may run without getting measurably closer to the end
     * of its path before the watchdog assumes something is wrong and cuts power.
     *
     * <p>Generous enough that a slow, heavily loaded, or briefly blocked robot is
     * not tripped, tight enough that nothing pushes into a wall for long.
     */
    private static final double NO_PROGRESS_TIMEOUT_SECONDS = 2.5;

    /** Inches of progress toward the endpoint that counts as "still working". */
    private static final double PROGRESS_EPSILON_INCHES = 0.5;

    private double speedMultiplier = 1.0;
    private boolean babyMode = false;

    /* Runaway watchdog state. */
    private double bestDistanceToEnd = Double.POSITIVE_INFINITY;
    private double lastProgressSeconds = 0.0;
    private boolean wasFollowing = false;
    private int lastPathIndex = -1;
    private String faultReason = null;

    public DriveSubsystem(HardwareMap hardwareMap, Pose startingPose) {
        super(hardwareMap, PedroConstants.createFollower(hardwareMap));
        follower.setPose(startingPose);
    }

    public DriveSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap, new Pose(0, 0, 0));
    }

    /* ===================== Teleop drive ===================== */

    /**
     * Puts the follower into manual mode with zero output, ready for driver input.
     *
     * <p>Pedro 3 has no explicit "start teleop" call -- the first {@code manual()}
     * switches mode -- but doing it once at start means the robot is not sitting
     * in a hold from whatever auto left behind.
     */
    public void startTeleOp() {
        follower.manual(0.0, 0.0, 0.0);
    }

    /**
     * Field-centric drive.
     *
     * @param forward +1 drives away from the driver station wall along +x
     * @param strafe  +1 drives to the left
     * @param turn    +1 turns counter-clockwise
     */
    public void driveFieldCentric(double forward, double strafe, double turn) {
        follower.manual(ManualDrive.fieldCentric(
                scale(forward),
                scale(strafe),
                scale(turn),
                follower.pose().heading()));
    }

    /** Same as {@link #driveFieldCentric} but relative to the robot's own nose. */
    public void driveRobotCentric(double forward, double strafe, double turn) {
        follower.manual(scale(forward), scale(strafe), scale(turn));
    }

    private double scale(double value) {
        double deadbanded = Math.abs(value) < RobotConstants.STICK_DEADBAND ? 0.0 : value;
        return deadbanded * speedMultiplier;
    }

    /*
     * stop() comes from PedroSubsystem and calls follower.stop(), which latches
     * the follower into IDLE. Pedro 3's update() calls drivetrain.stop() every
     * cycle while idle, so that genuinely cuts power and keeps it cut -- unlike
     * Pedro 2, where zeroing the teleop vector was ignored during path following.
     */

    /**
     * Caps follower speed as a fraction of the robot's maximum achievable
     * velocity, taking effect immediately.
     *
     * <p>Pedro 3 dropped {@code setMaxPower} and exposes the cap as a Foresight
     * {@code ConfigVar} instead. valleyLib reaches it through a protected hook,
     * and its own {@code setMaxSpeed} returns a Command -- convenient for a
     * routine, awkward for one-time setup -- so this sets it directly.
     *
     * @param maxSpeed fraction of maximum velocity, or
     *                 {@link PedroSubsystem#NO_SPEED_LIMIT} to remove the cap
     */
    public void applyMaxSpeed(double maxSpeed) {
        com.pedropathing.config.ConfigVar<Double> cap = maxPathSpeed();
        if (cap != null) {
            cap.set(maxSpeed);
        }
    }

    /**
     * Watchdog: cuts power when the follower is driving but getting nowhere.
     *
     * <p>A bad localizer sign, a mis-set pod offset, or a physical block all look
     * the same from here: the follower keeps commanding power and the distance to
     * the end of the path stops falling. Left alone that means four mecanum
     * motors stalled at full power for the rest of the OpMode, which is enough
     * current to brown out or damage a Control Hub. So the drivetrain is the
     * thing that has to notice, not the individual commands.
     *
     * <p>Once tripped the fault latches until the next path starts, so a command
     * cannot immediately re-drive into the same stall.
     */
    @Override
    public void periodic() {
        super.periodic();
        checkForRunaway();
    }

    private void checkForRunaway() {
        // Only path following is watched. A hold -- which is both an in-place turn
        // and what Pedro parks in at the end of a path -- has no endpoint to close
        // on, and is bounded by the timeout TurnToHeadingCommand carries.
        if (!follower.following()) {
            wasFollowing = false;
            return;
        }

        double now = RobotClock.seconds();

        // A new path resets the watchdog and clears any latched fault.
        if (!wasFollowing) {
            wasFollowing = true;
            lastPathIndex = -1;
            bestDistanceToEnd = Double.POSITIVE_INFINITY;
            lastProgressSeconds = now;
            faultReason = null;
        }

        // Pedro 3 dropped isLocalizationNAN(), so check the pose directly.
        Pose pose = follower.pose();
        if (Double.isNaN(pose.x()) || Double.isNaN(pose.y()) || Double.isNaN(pose.heading())) {
            trip("pose went NaN");
            return;
        }

        // Belt and braces: distanceToEndpoint() measures to the end of the whole
        // path, so it should fall monotonically, but re-baseline if the follower
        // reports a new segment rather than reading the jump as a stall.
        int pathIndex = follower.pathIndex();
        if (pathIndex != lastPathIndex) {
            lastPathIndex = pathIndex;
            bestDistanceToEnd = Double.POSITIVE_INFINITY;
            lastProgressSeconds = now;
        }

        double distance = follower.distanceToEndpoint();
        if (distance < bestDistanceToEnd - PROGRESS_EPSILON_INCHES) {
            bestDistanceToEnd = distance;
            lastProgressSeconds = now;
        } else if (now - lastProgressSeconds > NO_PROGRESS_TIMEOUT_SECONDS) {
            trip(String.format("no progress for %.1fs, %.1f in from path end",
                    now - lastProgressSeconds, distance));
        }
    }

    private void trip(String reason) {
        faultReason = reason;
        follower.stop();
        wasFollowing = false;
    }

    /** Non-null when the watchdog has cut power; cleared when the next path starts. */
    public String getFaultReason() {
        return faultReason;
    }

    public boolean hasFault() {
        return faultReason != null;
    }

    /* ===================== Baby mode ===================== */

    /** Precision driving: scales every teleop output down. */
    public void setBabyMode(boolean enabled) {
        babyMode = enabled;
        speedMultiplier = enabled ? RobotConstants.BABY_MODE_MULTIPLIER : 1.0;
    }

    public void toggleBabyMode() {
        setBabyMode(!babyMode);
    }

    public boolean isBabyMode() {
        return babyMode;
    }

    /* ===================== Pose and heading ===================== */

    /* getPose() comes from PedroSubsystem and returns follower.pose(). */

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public double getHeadingRadians() {
        return follower.pose().heading();
    }

    public double getHeadingDegrees() {
        return Math.toDegrees(getHeadingRadians());
    }

    /**
     * Zeroes the heading in place, keeping the current x/y.
     *
     * <p>Stands in for the old {@code imu.resetYaw()} -- the driver's "my
     * field-centric is crooked" button.
     */
    public void resetHeading() {
        follower.setHeading(0.0);
    }

    /**
     * Starts an in-place turn to an absolute field heading.
     *
     * <p>Pedro 3 has no {@code turnTo}: a turn is a hold of the current position
     * with a different heading.
     */
    public void turnTo(double headingRadians) {
        Pose current = follower.pose();
        follower.hold(new Pose(current.x(), current.y(), Angle.normalize(headingRadians)));
    }

    /** Starts an in-place turn of {@code deltaRadians} from where the robot is now. */
    public void turnBy(double deltaRadians) {
        turnTo(getHeadingRadians() + deltaRadians);
    }

    /** True while a commanded turn or end-of-path hold has not yet settled. */
    public boolean isTurning() {
        return follower.holding() && follower.isBusy();
    }
}
