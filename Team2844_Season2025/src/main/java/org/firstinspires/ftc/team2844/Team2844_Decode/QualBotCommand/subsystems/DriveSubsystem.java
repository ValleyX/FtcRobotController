package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.subsystems;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
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
    private int lastChainIndex = -1;
    private String faultReason = null;

    public DriveSubsystem(HardwareMap hardwareMap, Pose startingPose) {
        super(hardwareMap, PedroConstants.createFollower(hardwareMap));
        follower.setStartingPose(startingPose);
    }

    public DriveSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap, PedroConstants.ftcPose(0, 0, 0));
    }

    /* ===================== Teleop drive ===================== */

    /**
     * Hands the follower over to manual control. Call once when teleop starts;
     * path following and teleop drive are mutually exclusive modes.
     */
    public void startTeleOp() {
        follower.startTeleopDrive(true);
    }

    /**
     * Field-centric drive.
     *
     * @param forward +1 drives away from the driver station wall along +x
     * @param strafe  +1 drives to the left
     * @param turn    +1 turns counter-clockwise
     */
    public void driveFieldCentric(double forward, double strafe, double turn) {
        follower.setTeleOpDrive(
                deadband(forward) * speedMultiplier,
                deadband(strafe) * speedMultiplier,
                deadband(turn) * speedMultiplier,
                false);
    }

    /** Same as {@link #driveFieldCentric} but relative to the robot's own nose. */
    public void driveRobotCentric(double forward, double strafe, double turn) {
        follower.setTeleOpDrive(
                deadband(forward) * speedMultiplier,
                deadband(strafe) * speedMultiplier,
                deadband(turn) * speedMultiplier,
                true);
    }

    /**
     * Cuts drivetrain power for real.
     *
     * <p>Zeroing the teleop drive vector is not enough on its own: if the
     * follower is in path-following mode it ignores that vector entirely and
     * keeps driving. {@code breakFollowing()} zeroes and floats all four motors
     * and drops the follower out of whatever mode it was in.
     */
    public void stop() {
        follower.breakFollowing();
        follower.setTeleOpDrive(0, 0, 0, false);
    }

    /**
     * Watchdog: cuts power when the follower is driving but getting nowhere.
     *
     * <p>A bad localizer sign, a mis-set pod offset, or a physical block all look
     * the same from here: the follower stays busy, commands power, and the
     * distance to the end of the path stops falling. Left alone that means four
     * mecanum motors stalled at full power for the rest of the OpMode, which is
     * enough current to brown out or damage a Control Hub. So the drivetrain is
     * the thing that has to notice, not the individual commands.
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
        boolean following = follower.isBusy();

        if (!following) {
            wasFollowing = false;
            return;
        }

        double now = RobotClock.seconds();

        // A new path resets the watchdog and clears any latched fault.
        if (!wasFollowing) {
            wasFollowing = true;
            lastChainIndex = -1;
            bestDistanceToEnd = Double.POSITIVE_INFINITY;
            lastProgressSeconds = now;
            faultReason = null;
        }

        if (follower.isLocalizationNAN()) {
            trip("localizer returned NaN");
            return;
        }

        Pose pose = follower.getPose();
        if (Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            trip("pose went NaN");
            return;
        }

        // An in-place turn is a hold, not a path: there is no endpoint to close
        // on, so the progress test does not apply. Turns are bounded by the
        // timeout TurnToHeadingCommand carries instead.
        if (follower.isTurning() || follower.getCurrentPath() == null) {
            lastProgressSeconds = now;
            return;
        }

        // Each leg of a chain has its own endpoint, so the distance jumps up when
        // the follower advances. Re-baseline instead of reading that as a stall.
        int chainIndex = follower.getChainIndex();
        if (chainIndex != lastChainIndex) {
            lastChainIndex = chainIndex;
            bestDistanceToEnd = Double.POSITIVE_INFINITY;
            lastProgressSeconds = now;
        }

        double distance = distanceToPathEnd(pose);
        if (distance < bestDistanceToEnd - PROGRESS_EPSILON_INCHES) {
            bestDistanceToEnd = distance;
            lastProgressSeconds = now;
        } else if (now - lastProgressSeconds > NO_PROGRESS_TIMEOUT_SECONDS) {
            trip(String.format("no progress for %.1fs, %.1f in from path end",
                    now - lastProgressSeconds, distance));
        }
    }

    private double distanceToPathEnd(Pose pose) {
        Pose end = follower.getCurrentPath().getLastControlPoint();
        return Math.hypot(end.getX() - pose.getX(), end.getY() - pose.getY());
    }

    private void trip(String reason) {
        faultReason = reason;
        follower.breakFollowing();
        wasFollowing = false;
    }

    /** Non-null when the watchdog has cut power; cleared when the next path starts. */
    public String getFaultReason() {
        return faultReason;
    }

    public boolean hasFault() {
        return faultReason != null;
    }

    private static double deadband(double value) {
        return Math.abs(value) < RobotConstants.STICK_DEADBAND ? 0.0 : value;
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

    public Pose getPose() {
        return follower.getPose();
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public double getHeadingRadians() {
        return follower.getPose().getHeading();
    }

    public double getHeadingDegrees() {
        return Math.toDegrees(getHeadingRadians());
    }

    /**
     * Zeroes the heading in place, keeping the current x/y.
     *
     * <p>Stands in for the old {@code imu.resetYaw()} — the driver's "my
     * field-centric is crooked" button.
     */
    public void resetHeading() {
        Pose current = follower.getPose();
        follower.setPose(new Pose(current.getX(), current.getY(), 0.0, current.getCoordinateSystem()));
    }

    /** Starts an in-place turn to an absolute field heading. */
    public void turnTo(double headingRadians) {
        follower.turnTo(MathFunctions.normalizeAngle(headingRadians));
    }

    /** Starts an in-place turn of {@code deltaRadians} from where the robot is now. */
    public void turnBy(double deltaRadians) {
        turnTo(getHeadingRadians() + deltaRadians);
    }

    public boolean isTurning() {
        return follower.isTurning();
    }
}
