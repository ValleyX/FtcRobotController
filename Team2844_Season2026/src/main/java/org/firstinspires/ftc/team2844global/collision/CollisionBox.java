package org.firstinspires.ftc.team2844global.collision;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.team2844global.GlobalConstants;

/**
 * A box (or any convex shape) made from its corners. Works when the box is rotated.
 * <p>The corners must be listed in order going around the box (clockwise or counterclockwise),
 * not criss-crossed. Only x and y are used, the heading of each Pose is ignored.
 * Touching counts as colliding.
 */
public class CollisionBox implements Collider{
    private Pose[] corners;

    public CollisionBox(Pose[] corners){
        this.corners = corners;
    }

    public CollisionBox(Pose corner1, Pose corner2, Pose corner3, Pose corner4){
        this.corners = new Pose[]{corner1, corner2, corner3, corner4};
    }

    /**
     * True if this box and c overlap or touch.
     * <p>Uses the separating axis test: two convex shapes miss each other only if there is a gap
     * between them along the normal of one of their edges, so every edge of both boxes is checked.
     */
    @Override
    public boolean collides(CollisionBox c) {
        return !hasSeparatingEdge(corners, c.getCorners())
                && !hasSeparatingEdge(c.getCorners(), corners);
    }

    /**
     * True if c overlaps or touches this box: either the circle's center is inside the box,
     * or some edge of the box comes within the circle's radius of the center.
     */
    @Override
    public boolean collides(CollisionCircle c) {
        Pose center = c.getCenterPoint();
        if(contains(center)){
            return true;
        }
        for(int i = 0; i < corners.length; i++){
            Pose a = corners[i];
            Pose b = corners[(i + 1) % corners.length];
            if(distanceToEdge(center, a, b) <= c.getRadius()){
                return true;
            }
        }
        return false;
    }

    /**
     * True if the point is inside the box or on its edge.
     * <p>For a convex shape, the point is inside when it is on the same side of every edge.
     */
    public boolean contains(Pose point){
        boolean anyLeft = false;
        boolean anyRight = false;
        for(int i = 0; i < corners.length; i++){
            Pose a = corners[i];
            Pose b = corners[(i + 1) % corners.length];
            //cross product: positive when the point is left of edge a->b, negative when right
            double cross = (b.x() - a.x()) * (point.y() - a.y())
                    - (b.y() - a.y()) * (point.x() - a.x());
            if(cross > 0) anyLeft = true;
            if(cross < 0) anyRight = true;
        }
        return !(anyLeft && anyRight);
    }

    public Pose[] getCorners(){
        return corners;
    }

    public void setCorners(Pose[] corners) {
        this.corners = corners;
    }

    /**
     * True if one of shape's edges has a gap between shape and other along that edge's normal,
     * which means the two shapes cannot be touching.
     */
    private static boolean hasSeparatingEdge(Pose[] shape, Pose[] other){
        for(int i = 0; i < shape.length; i++){
            Pose a = shape[i];
            Pose b = shape[(i + 1) % shape.length];
            //normal (perpendicular) to edge a->b
            double axisX = -(b.y() - a.y());
            double axisY = b.x() - a.x();

            //the "shadow" each shape casts on the normal
            double[] shapeRange = project(shape, axisX, axisY);
            double[] otherRange = project(other, axisX, axisY);

            if(shapeRange[1] < otherRange[0] || otherRange[1] < shapeRange[0]){
                return true;
            }
        }
        return false;
    }

    /**Returns {min, max} of the corners projected onto the axis (dot product).*/
    private static double[] project(Pose[] shape, double axisX, double axisY){
        double min = Double.POSITIVE_INFINITY;
        double max = Double.NEGATIVE_INFINITY;
        for(Pose p : shape){
            double d = p.x() * axisX + p.y() * axisY;
            min = Math.min(min, d);
            max = Math.max(max, d);
        }
        return new double[]{min, max};
    }

    /**The point on the edge of this box closest to point. If point is inside the box, this is the closest point on the edge.*/
    public Pose closestPoint(Pose point){
        Pose closest = corners[0];
        double closestDistance = Double.POSITIVE_INFINITY;
        for(int i = 0; i < corners.length; i++){
            Pose onEdge = closestPointOnEdge(point, corners[i], corners[(i + 1) % corners.length]);
            double distance = Math.hypot(point.x() - onEdge.x(), point.y() - onEdge.y());
            if(distance < closestDistance){
                closestDistance = distance;
                closest = onEdge;
            }
        }
        return closest;
    }

    /**Shortest distance from point to the line segment a->b.*/
    private static double distanceToEdge(Pose point, Pose a, Pose b){
        Pose closest = closestPointOnEdge(point, a, b);
        return Math.hypot(point.x() - closest.x(), point.y() - closest.y());
    }

    /**The point on the line segment a->b closest to point.*/
    private static Pose closestPointOnEdge(Pose point, Pose a, Pose b){
        double edgeX = b.x() - a.x();
        double edgeY = b.y() - a.y();
        double lengthSquared = edgeX * edgeX + edgeY * edgeY;

        //how far along the edge the closest point is, 0 at a and 1 at b
        double t = 0.0;
        if(lengthSquared > 0.0){
            t = ((point.x() - a.x()) * edgeX + (point.y() - a.y()) * edgeY) / lengthSquared;
            t = Math.max(0.0, Math.min(1.0, t));
        }
        return new Pose(a.x() + t * edgeX, a.y() + t * edgeY);
    }
}
