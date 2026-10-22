package org.firstinspires.ftc.team2844.subsystems;

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
     * Field-centric drive. Automatically corrects for red and blue by reversing the drivers controls
     *
     * @param forward +1 drives away from the driver station wall along +x
     * @param strafe  +1 drives to the left
     * @param turn    +1 turns counter-clockwise
     */
    public void driveFieldCentric(double forward, double strafe, double turn) {
        double offsetHeading = Constants.alliance == Constants.BLUE_ALLIANCE? Math.PI: 0.0;
        follower.manual(ManualDrive.fieldCentric(
                scale(forward),
                scale(strafe),
                scale(turn),
                follower.pose().heading(),
                offsetHeading));
    }

    /** Same as {@link #driveFieldCentric} but relative to the robot's own nose. */
    public void driveRobotCentric(double forward, double strafe, double turn) {
        follower.manual(scale(forward), scale(strafe), scale(turn));
    }

    private double scale(double value) {
        double deadBanded = Math.abs(value) < Constants.STICK_DEADBAND ? 0.0 : value;
        return deadBanded * speedMultiplier;
    }


    /**
     * Calculates the 3d distance from the bot to the bottom of the tipped up cell of the alliances color.
     * Assumes Constants.changeTarget() has been updated recently and odometry tracking is accurate
     * */
    public double getDistance(){
        Pose pose = follower.pose();
        return Math.sqrt(Math.pow(pose.x() - Constants.targetX, 2) + Math.pow(pose.y() - Constants.targetY, 2) + Math.pow(Constants.CELL_HEIGHT - Constants.LL_HEIGHT, 2));
    }



    /* ===================== Pose and heading ===================== */

    /* getPose() comes from PedroSubsystem and returns follower.pose(). */

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    /**Returns a value 0 to 2PI*/
    public double getHeadingRadians() {
        return follower.pose().heading();
    }

    /**Returns a value 0 to 360*/
    public double getHeadingDegrees() {
        return Math.toDegrees(getHeadingRadians());
    }

    /**Returns a value -180 to 180*/
    public double getNormalHeadingDegrees(){
        return normalizeDegrees(Math.toDegrees(getHeadingRadians()));
    }

    /**Returns a value -PI to PI*/
    public double getNormalHeadingRadians(){
        return normalizeRadians(getHeadingRadians());
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

    /**Converts 0 to 360 into -180 to 180*/
    private double normalizeDegrees(double degrees){
        double temp = degrees;
        while (temp>180) temp-=360;
        while (temp<-180) temp+=360;
        return temp;
    }

    /**Converts 0 to 2PI into -PI to PI*/
    private double normalizeRadians(double radians){
        double temp = radians;
        while (temp>Math.PI) temp-= 2*Math.PI;
        while (temp<-Math.PI) temp+= 2*Math.PI;
        return temp;
    }

}
