package org.firstinspires.ftc.team2844global;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.team2844global.collision.CollisionBox;
import org.firstinspires.ftc.team2844global.collision.CollisionCircle;

public class Ball {
    private static final double efficiency = 0.6;
    private static int ballID = 0;
    /**How fast a rolling pollen/nectar slows down, in inches/s^2. Rolling friction is about the same
     * at any speed, so this does not change with velocity.*/
    private static final double ROLLING_DECEL = 11.0; //TODO measure: roll a ball, ROLLING_DECEL = 2 * inches rolled / (seconds until it stops)^2
    /**A bounce closer than this to a box corner, in inches, can't be predicted: a tiny error in where the ball hits
     * changes which way it goes. predictEndPose gives up on those balls.*/
    private static final double CORNER_MARGIN = 1.0; //TODO tune: make it bigger if the camera data is noisy
    /**Returns the normal acceleration for a pollen/nectar in inches/s^2: ROLLING_DECEL pointed opposite the velocity.*/
    public static Vector2D getAveAccel(Vector2D vel){
        double speed = Math.hypot(vel.x(), vel.y());
        if(speed == 0.0){
            return Vector2D.cartesian(0.0, 0.0);
        }
        return Vector2D.cartesian(-ROLLING_DECEL*vel.x()/speed, -ROLLING_DECEL*vel.y()/speed);
    }
    private Pose currentPose;
    private final Pose initialPose;
    private final CollisionCircle collision;
    private final double ballRadius;
    private final int colorID;
    private int trackID;
    private Vector2D currentVel;
    private Vector2D initialVel;
    private Vector2D acceleration;
    private final ElapsedTime velTimer;

    public Ball(Pose initialPose, int colorID, Vector2D initialVelocity, Vector2D acceleration){
        this.currentPose = initialPose;
        this.initialPose = initialPose;
        this.colorID = colorID;
        this.currentVel = initialVelocity;
        this.initialVel = initialVelocity;
        this.acceleration = acceleration;

        trackID = ballID;

        ballRadius = colorID == GlobalConstants.POLLEN ? 1.4: 1.8;
        collision = new CollisionCircle(initialPose, ballRadius);

        velTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        velTimer.reset();

        ballID++;
    }

    public Ball(Pose initialPose, int colorID, Vector2D initialVelocity){
        this(initialPose, colorID, initialVelocity, null);
    }

    public Ball(Pose initialPose, int colorID){
        this(initialPose, colorID, null);
    }

    /* ---------- SETTERS ---------- */
    public void setTrackID(int trackID){
        this.trackID = trackID;
    }

    public void setPose(Pose newPose){
        currentPose = newPose;
        collision.setCenterPoint(currentPose);

        setVelocityAcceleration(Vector2D.cartesian(
                (currentPose.x() - initialPose.x())/velTimer.seconds(),
                ((currentPose.y() - initialPose.y())/velTimer.seconds())
        ));
    }

    /**Moves this ball to where newerBall was seen, and sets the velocity to how fast it was going at that moment.
     * The time used is the time between when this ball and newerBall were seen, so a ball that went out of view
     * early still gets the right speed.
     * <p>Distance over time gives the average velocity, which is the velocity halfway between the two sightings.
     * The ball keeps slowing down by ROLLING_DECEL for the second half, so that is taken off the speed.
     * <p>Does nothing if newerBall is this ball or was not seen after it.*/
    public void setPose(Ball newerBall){
        double secondsBetween = velTimer.seconds() - newerBall.velTimer.seconds();
        if(secondsBetween <= 0.0){
            return;
        }
        currentPose = newerBall.getPose();
        collision.setCenterPoint(currentPose);

        double averageVelX = (currentPose.x() - initialPose.x())/secondsBetween;
        double averageVelY = (currentPose.y() - initialPose.y())/secondsBetween;
        double averageSpeed = Math.hypot(averageVelX, averageVelY);
        //if this goes below 0, the ball stopped before it was last seen
        double lastSpeed = Math.max(0.0, averageSpeed - ROLLING_DECEL*secondsBetween/2.0);
        //shrinks the velocity to lastSpeed without changing its direction
        double shrink = (averageSpeed == 0.0)? 0.0: lastSpeed/averageSpeed;

        setVelocityAcceleration(Vector2D.cartesian(averageVelX*shrink, averageVelY*shrink));
    }

    public void setVelocityAcceleration(Vector2D newVel){
        if(currentVel == null){
            currentVel = newVel;
            initialVel = currentVel;
        } else {
            currentVel = newVel;

            acceleration = getAveAccel(currentVel);
        }
    }

    /*! ----- PREDICTOR ----- !*/
    /**Where the ball will stop, bouncing off the walls and obstacles on the way.
     * <p>Returns null if it bounces within CORNER_MARGIN of a box corner, because where it goes after that
     * can't be predicted. A later snapshot will see it once it stops.*/
    public Pose predictEndPose(){
        Pose endPose;
        if(currentVel == null){
            return currentPose;
        } else if (acceleration == null){
            acceleration = getAveAccel(currentVel);
            endPose = calcEndPose(currentPose, currentVel, acceleration);
        } else {
            endPose = calcEndPose(currentPose, currentVel, acceleration);
        }

        int loops = 5;
        Path ballPath;
        Pose collisionPose;
        Pose startingPose = currentPose;
        Vector2D startingVel = Vector2D.cartesian(currentVel.x(), currentVel.y());
        do {
            //Pedro can't make a line with no length, and a ball that doesn't move can't hit anything
            if(startingPose.distance(endPose) < 0.01){
                return endPose;
            }
            ballPath = Paths.line(startingPose, endPose).tangent();
            Vector2D accel = getAveAccel(startingVel);
            collisionPose = collision.collisionInPathPose(GlobalConstants.ALL_COLLISIONS, ballPath);
            if (collisionPose != null) {
                //collisionPose is already up to 1 inch into the wall, so find where the ball first touches it.
                //The next leg starts there, so it doesn't hit the same wall again right away.
                Pose touchPose = findTouchPose(startingPose, endPose, collisionPose);
                double collisionTime = calcTimeAtPose(touchPose, startingPose, startingVel, accel);
                if (collisionTime == GlobalConstants.NO_TIME) {
                    //only happens when the ball stops right as it touches the wall
                    return touchPose;
                }
                double xVelAtCollision = calcVelAtTime(startingVel.x(), accel.x(), collisionTime);
                double yVelAtCollision = calcVelAtTime(startingVel.y(), accel.y(), collisionTime);
                //the box it bounces off, and the closest point on it
                CollisionBox hitBox = bounceBox(touchPose, xVelAtCollision, yVelAtCollision);
                if (hitBox == null) {
                    //it's only rolling along something it touches, so it doesn't bounce
                    return endPose;
                }
                Pose nearestPoint = hitBox.closestPoint(touchPose);
                //where it goes after hitting a corner depends on exactly where it hits, which the camera can't see well enough
                if (nearFreeCorner(hitBox, nearestPoint)) {
                    return null;
                }
                //bounce off the surface it hit: the normal points from the closest point on the box to the ball's center.
                //The part of the velocity going into the box is reversed and loses speed, the part along the box stays the same.
                //On a flat face the normal is straight out, so this just flips x or y.
                double gap = touchPose.distance(nearestPoint);
                double normalX = (touchPose.x() - nearestPoint.x())/gap;
                double normalY = (touchPose.y() - nearestPoint.y())/gap;
                double intoWall = xVelAtCollision*normalX + yVelAtCollision*normalY;
                double xVelAfterCollision = xVelAtCollision - (1.0 + efficiency)*intoWall*normalX;
                double yVelAfterCollision = yVelAtCollision - (1.0 + efficiency)*intoWall*normalY;
                startingVel = Vector2D.cartesian(xVelAfterCollision, yVelAfterCollision);
                endPose = calcEndPose(touchPose, startingVel, getAveAccel(startingVel));
                startingPose = touchPose;
            }
            loops--;
        } while (collisionPose != null && loops >= 0);

        //ran out of loops while it was still bouncing, and the last leg was never checked: the last wall it touched is the best guess
        if (collisionPose != null) {
            return startingPose;
        }
        return endPose;
    }

    /**Where the ball first touches something on the way from legStart to hitPose.
     * collisionInPathPose checks every inch, so the point 1 inch before hitPose was clear. This halves the gap
     * between that clear point and hitPose until it is less than a ten-thousandth of an inch.*/
    private Pose findTouchPose(Pose legStart, Pose legEnd, Pose hitPose){
        double legLength = legStart.distance(legEnd);
        double directionX = (legEnd.x() - legStart.x())/legLength;
        double directionY = (legEnd.y() - legStart.y())/legLength;
        double hitDistance = legStart.distance(hitPose);
        double clearDistance = Math.max(0.0, hitDistance - 1.0);
        for (int i = 0; i < 15; i++) {
            double middleDistance = (clearDistance + hitDistance)/2.0;
            Pose middlePose = new Pose(legStart.x() + directionX*middleDistance, legStart.y() + directionY*middleDistance);
            if (collision.collidesAtPose(GlobalConstants.ALL_COLLISIONS, middlePose)) {
                hitDistance = middleDistance;
            } else {
                clearDistance = middleDistance;
            }
        }
        return new Pose(legStart.x() + directionX*clearDistance, legStart.y() + directionY*clearDistance);
    }

    /**The box the ball bounces off when it is at pose with this velocity: of the boxes it is touching, the one it is
     * moving into the fastest. In a corner where two walls meet it touches both, and this picks one; the next pass of the
     * loop bounces it off the other. Returns null if it isn't moving into anything it touches.*/
    private CollisionBox bounceBox(Pose pose, double velX, double velY){
        CollisionBox bounceOff = null;
        double fastestInto = 0.0;
        for (CollisionBox box : GlobalConstants.ALL_COLLISIONS) {
            Pose nearest = box.closestPoint(pose);
            double gap = pose.distance(nearest);
            //not touching this one (findTouchPose leaves the ball a tiny bit away from the box)
            if (gap > ballRadius + 0.01 || gap < 1e-6) {
                continue;
            }
            //speed toward the box: negative when moving into it
            double into = (velX*(pose.x() - nearest.x()) + velY*(pose.y() - nearest.y()))/gap;
            if (into < fastestInto) {
                fastestInto = into;
                bounceOff = box;
            }
        }
        return bounceOff;
    }

    /**True if point is within CORNER_MARGIN of one of box's corners.
     * Corners on or outside the field edge don't count: those are the ends of the walls, or where a flower meets a wall,
     * and a ball can't hit them.*/
    private static boolean nearFreeCorner(CollisionBox box, Pose point){
        for (Pose corner : box.getCorners()) {
            boolean onFieldEdge = corner.x() <= 0.0 || corner.x() >= GlobalConstants.FIELD_SIZE
                    || corner.y() <= 0.0 || corner.y() >= GlobalConstants.FIELD_SIZE;
            if (!onFieldEdge && point.distance(corner) < CORNER_MARGIN) {
                return true;
            }
        }
        return false;
    }

    /* ---------- CALCS ---------- */

    private Pose calcEndPose(Pose startPose, Vector2D vel, Vector2D accel){
        if(accel == null || (accel.x() == 0.0 && accel.y() == 0.0)){
            return startPose;
        }

        double x;
        double y;
        if(accel.x() != 0.0) {
            double endTimeX = (-vel.x()) / (accel.x());
            x = startPose.x() + (vel.x() * endTimeX) + (0.5 * accel.x() * Math.pow(endTimeX, 2));
        } else {
            x = startPose.x();
        }

        if(accel.y() != 0.0) {
            double endTimeY = (-vel.y()) / (accel.y());
            y = startPose.y() + (vel.y() * endTimeY) + (0.5 * accel.y() * Math.pow(endTimeY, 2));
        } else {
            y = startPose.y();
        }
        return new Pose(x, y);
    }

    private double calcVelAtTime(double initialVel, double accel, double time){
        return (initialVel) + accel*time;
    }

    /**Seconds from now until the ball reaches p, or NO_TIME if it never gets there.
     * The ball rolls in a straight line, so x and y give the same time. This uses whichever axis
     * the ball is moving faster along, because an axis with no velocity can't give a time.*/
    private double calcTimeAtPose(Pose p){
        if (currentVel == null){
            return GlobalConstants.NO_TIME;
        } else if (acceleration == null){
            acceleration = getAveAccel(currentVel);
        }

        boolean useX = Math.abs(currentVel.x()) >= Math.abs(currentVel.y());
        double vel = useX ? currentVel.x() : currentVel.y();
        double accel = useX ? acceleration.x() : acceleration.y();
        double distance = useX ? p.x() - currentPose.x() : p.y() - currentPose.y();

        if(vel == 0.0){
            return GlobalConstants.NO_TIME;
        }

        double time;
        if(accel == 0.0){
            time = distance/vel;
        } else {
            double discriminant = Math.pow(vel, 2) + 2*accel*distance;
            if(discriminant < 0.0){
                return GlobalConstants.NO_TIME;
            }
            //the +/- matches the sign of the velocity so this is the first time it gets there, not the second
            time = (-vel + Math.signum(vel)*Math.sqrt(discriminant))/accel;
        }

        if(time < 0.0){
            return GlobalConstants.NO_TIME;
        }
        return time;
    }

    /**Seconds from now until the ball reaches p, or NO_TIME if it never gets there.
     * The ball rolls in a straight line, so x and y give the same time. This uses whichever axis
     * the ball is moving faster along, because an axis with no velocity can't give a time.*/
    private double calcTimeAtPose(Pose targetP, Pose currentPose, Vector2D currentVel, Vector2D acceleration){
        if (currentVel == null){
            return GlobalConstants.NO_TIME;
        } else if (acceleration == null){
            acceleration = getAveAccel(currentVel);
        }

        boolean useX = Math.abs(currentVel.x()) >= Math.abs(currentVel.y());
        double vel = useX ? currentVel.x() : currentVel.y();
        double accel = useX ? acceleration.x() : acceleration.y();
        double distance = useX ? targetP.x() - currentPose.x() : targetP.y() - currentPose.y();

        if(vel == 0.0){
            return GlobalConstants.NO_TIME;
        }

        double time;
        if(accel == 0.0){
            time = distance/vel;
        } else {
            double discriminant = Math.pow(vel, 2) + 2*accel*distance;
            if(discriminant < 0.0){
                return GlobalConstants.NO_TIME;
            }
            //the +/- matches the sign of the velocity so this is the first time it gets there, not the second
            time = (-vel + Math.signum(vel)*Math.sqrt(discriminant))/accel;
        }

        if(time < 0.0){
            return GlobalConstants.NO_TIME;
        }
        return time;
    }

    public double distance(Ball ball){
        return distance(ball.getPose());
    }

    public double distance(Pose pose){
        return Math.hypot(currentPose.x() - pose.x(), currentPose.y() - pose.y());
    }



    /* ---------- GETTERS ---------- */
    public Vector2D getVelocity(){
        return currentVel;
    }

    public Pose getPose(){
        return currentPose;
    }

    public double getBallRadius(){
        return ballRadius;
    }

    public Vector2D getAcceleration(){
        return acceleration;
    }

    public int getColorID(){
        return colorID;
    }

    public CollisionCircle getCollision(){
        return collision;
    }

    /** The tracking ID for this specific ball, or the ball that it is connected to*/
    public int getTrackID(){
        return trackID;
    }

    /**The total amount of Ball objects made since the robot has restarted*/
    public int getBallID(){
        return ballID;
    }
}
