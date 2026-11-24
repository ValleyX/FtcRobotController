package org.firstinspires.ftc.team2844simple.subsystems;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vcs.valleylib.ftc.pedro.PedroSubsystem;

import org.firstinspires.ftc.team2844.helpers.collision.CollisionBox;
import org.firstinspires.ftc.team2844simple.helpers.Constants;
import org.firstinspires.ftc.team2844simple.pedro.PedroConstants;

import java.util.ArrayList;
import java.util.List;

public class DriveSubsystem extends PedroSubsystem {

    private double speedMultiplier = 1.0;

    public DriveSubsystem(HardwareMap hardwareMap, Pose startingPose) {
        super(hardwareMap, PedroConstants.createFollower(hardwareMap));
        follower.setPose(startingPose);
    }




    /* -------------------- Teleop drive -------------------- */

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
        follower.manual(ManualDrive.fieldCentric(
                scale(forward),
                scale(strafe),
                scale(turn),
                follower.pose().heading(),
                allianceHeadingOffset()));
    }

    /**
     * Field-centric drive where the turn is a computed power (for example from an aim controller)
     * instead of a joystick. Forward and strafe get the same deadband and speed multiplier as
     * {@link #driveFieldCentric}; the turn does not, so small corrections are not deadbanded away.
     *
     * @param forward   +1 drives away from the driver station wall along +x
     * @param strafe    +1 drives to the left
     * @param turnPower +1 turns counter-clockwise, clipped to -1 to 1
     */
    public void driveFieldCentricWithTurnPower(double forward, double strafe, double turnPower) {
        follower.manual(ManualDrive.fieldCentric(
                scale(forward),
                scale(strafe),
                Math.max(-1.0, Math.min(1.0, turnPower)),
                follower.pose().heading(),
                allianceHeadingOffset()));
    }

    /** Field-centric offset so each alliance's driver pushes "forward" away from their own wall. */
    private double allianceHeadingOffset() {
        return Constants.alliance == Constants.BLUE_ALLIANCE? Math.PI: 0.0;
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


    /* ===================== Aiming at the target ===================== */

    /** True once Constants.changeTarget() has set a target position. */
    public boolean hasTarget(){
        return Constants.targetX != Constants.NO_POS && Constants.targetY != Constants.NO_POS;
    }

    /**
     * Field heading (radians, -PI to PI) the robot has to face for its front to point at
     * the current target (Constants.targetX / targetY). The shooter is on the front of the robot.
     */
    public double getTargetHeadingRadians(){
        Pose pose = follower.pose();
        return Math.atan2(Constants.targetY - pose.y(), Constants.targetX - pose.x());
    }

    /**
     * How far the robot has to turn to face the target, in radians from -PI to PI.
     * Positive means turn counter-clockwise. Returns 0 if there is no target yet.
     */
    public double getHeadingErrorToTarget(){
        if(!hasTarget()) return 0.0;
        return normalizeRadians(getTargetHeadingRadians() - getHeadingRadians());
    }





    /* ---------- Automatic Pathing ---------- */

    private List<Pose> routeAroundCollision(Pose startPose, Pose endPose, int depth){
        if(depth <= 0)
            return null;
        //check for collision from start point to endpoint
        List<Pose> poses = new ArrayList<Pose>();

        Path p = Paths.line(startPose, endPose).tangent();
        int collision = collisionInPath(p);
        if(collision >= 0){
            //if so, look for paths around collision
            Pose newStart = findCorner(collision, startPose, endPose);
            if(newStart == null)
                return null;
            //run again with new start point
            List<Pose> list = routeAroundCollision(newStart, endPose, depth-1);

            if(list != null) {
                poses.add(startPose);
                poses.addAll(list);
                return poses;
            } else {
                return null;
            }
        } else {
            //if no collision, return poses
            poses.add(startPose);
            poses.add(endPose);
        }
        return poses;
    }

    private boolean possibleCorners(int j, Pose startPose){
        if(j == -1){
            return true;
        }

        Pose[] corners = Constants.COLLISIONS[j].getCorners();

        for(int i = 0; i < corners.length; i++){
            if(!corners[i].equals(startPose) && (collisionInPath(Paths.line(startPose, corners[i]).tangent()) < 0)){
                return true;
            }
        }

        return false;
    }

    private Pose findCorner(int j, Pose startPose, Pose endPose){
        Pose[] corners = Constants.SAFE_COLLISIONS[j].getCorners().clone();

        List<Pose> poses = new ArrayList<Pose>();
        for(int i = 0; i < corners.length; i++){
            if(!corners[i].equals(startPose) && (collisionInPath(Paths.line(startPose, corners[i]).tangent()) < 0)){
                poses.add(corners[i]);
            }
        }

        if(poses.isEmpty())
            return null;

        double minLength = lineLength(startPose, poses.get(0), endPose);
        int index = 0;
        for (int k = 1; k < poses.size(); k++){
            double check = lineLength(startPose, poses.get(k), endPose);
            if(check < minLength){
                minLength = check;
                index = k;
            }
        }
        return poses.get(index);
    }

    private double lineLength(Pose startPose, Pose middlePose, Pose endPose){
        double startLength = Math.hypot(startPose.x() - middlePose.x(), startPose.y() - middlePose.y());
        double endLength = Math.hypot(middlePose.x() - endPose.x(), middlePose.y() - endPose.y());
        return startLength + endLength;
    }

    private Path pathify(List<Pose> poses){
        List<Path> curveList = new ArrayList<Path>();
        for(int i = 1; i < poses.size()-1; i++){
            Pose[] curves = makeCurvy(poses.get(i-1), poses.get(i), poses.get(i+1));
            curveList.add(Paths.curve(curves[0], curves[1], curves[2]));
        }

        List<Path> pathList = new ArrayList<Path>();
        Pose start = poses.get(0);
        for(int k = 0; k < curveList.size(); k++){
            pathList.add(Paths.line(start, curveList.get(k).curve.startPoint().toPose()).tangent());
            pathList.add(curveList.get(k));
            start = curveList.get(k).curve.endPoint().toPose();
        }
        pathList.add(Paths.line(start, poses.get(poses.size()-1)).tangent());

        return combinePaths(pathList);
    }

    private Path combinePaths(List<Path> paths){
        return Paths.path(paths.toArray(new Path[0])).tangent();
    }

    private Pose[] makeCurvy(Pose previous, Pose mid, Pose next){
        double inX = mid.x() - previous.x();
        double inY = mid.y() - previous.y();
        double outX = next.x() - mid.x();
        double outY = next.y() - mid.y();
        double inLength = Math.hypot(inX, inY);
        double outLength = Math.hypot(outX, outY);

        double inDirX = inX/inLength;
        double inDirY = inY/inLength;
        double outDirX = outX/outLength;
        double outDirY = outY/outLength;

        double roundDistance = Math.min(Constants.MAX_CURVE_DIS, Math.min(0.4 * inLength, 0.4 * outLength));

        return new Pose[]{
                new Pose(mid.x()-inDirX*roundDistance, mid.y() - inDirY*roundDistance),
                mid,
                new Pose(mid.x()+outDirX*roundDistance, mid.y() + outDirY*roundDistance)
        };
    }

    /**Returns the ID of the CollisionBox that the robot hits along the given path. Returns -1  if it does not collide with anything*/
    private int collisionInPath(Path p){
        p = p.tangent();
        double maxLength = p.curve.length();
        for(int i = 1; i <= (int) Math.ceil(maxLength); i++){
            double t = Math.min(Math.max(i/maxLength, 0), 1);
            for(int j = 0; j < Constants.COLLISIONS.length; j++){
                if(Constants.COLLISIONS[j].collides(getRobotCollision(p.get(t)))){
                    return j;
                }
            }
        }
        return -1;
    }

    public Path autoRoute(Pose startPose, Pose endPose) throws NullPointerException{
        //check for collision in straight path
        //if collides, find path to spot that won't collide then;
        List<Pose> poses = routeAroundCollision(startPose, endPose, Constants.DEFAULT_DEPTH);
        return poses != null ? pathify(poses): null;
    }

    /**returns a CollisionBox of the robot with a safety margin given the current pose*/
    public CollisionBox getRobotCollision(Pose pose){
        double h = pose.heading();
        double x = pose.x();
        double y = pose.y();

        double halfLength = (Constants.ROBOT_LENGTH/2.0)+Constants.SAFETY_MARGIN;
        double halfWidth = (Constants.ROBOT_WIDTH/2.0)+Constants.SAFETY_MARGIN;

        double fx = Math.cos(h)*halfLength;
        double fy = Math.sin(h)*halfLength;
        double lx = -Math.sin(h)*halfWidth;
        double ly = Math.cos(h)*halfWidth;
        return new CollisionBox(
                new Pose(x - fx - lx, y - fy - ly),
                new Pose(x + fx - lx, y + fy - ly),
                new Pose(x + fx + lx, y + fy + ly),
                new Pose(x - fx + lx, y - fy + ly)
        );
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
     */
    public void resetHeading() {
        follower.setHeading((Constants.alliance == Constants.BLUE_ALLIANCE)
                ? Math.PI
                : 0.0
        );
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

    public void holdPosition(Pose hold){
        follower.hold(hold);
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
