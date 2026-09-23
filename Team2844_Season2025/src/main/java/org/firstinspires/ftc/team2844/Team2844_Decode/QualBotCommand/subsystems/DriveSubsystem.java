package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.subsystems;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.HardwareMap;
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

    private double speedMultiplier = 1.0;
    private boolean babyMode = false;

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

    public void stop() {
        follower.setTeleOpDrive(0, 0, 0, false);
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
