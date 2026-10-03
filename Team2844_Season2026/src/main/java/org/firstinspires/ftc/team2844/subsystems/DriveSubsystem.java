package org.firstinspires.ftc.team2844.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vcs.valleylib.ftc.pedro.PedroSubsystem;

import org.firstinspires.ftc.team2844.helpers.Constants;
import org.firstinspires.ftc.team2844.pedro.PedroConstants;

public class DriveSubsystem extends PedroSubsystem {

    private double speedMultiplier = 1.0;

    public DriveSubsystem(HardwareMap hardwareMap, Pose startingPose) {
        super(hardwareMap, PedroConstants.createFollower(hardwareMap));
        follower.setPose(startingPose);
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
        double deadBanded = Math.abs(value) < Constants.STICK_DEADBAND ? 0.0 : value;
        return deadBanded * speedMultiplier;
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
