package org.firstinspires.ftc.team2844global.collision;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class CollisionCircle implements Collider{
    private Pose centerPoint;
    private double radius;
    public CollisionCircle(Pose centerPoint, double radius){
        this.centerPoint = centerPoint;
        this.radius = radius;
    }
    @Override
    public boolean collides(CollisionBox c) {
        //same check either way around, so let the box do it
        return c.collides(this);
    }

    @Override
    public boolean collides(CollisionCircle c) {
        return distance(getCenterPoint(), c.getCenterPoint())
                <= getRadius() + c.getRadius();
    }

    public boolean collidesAtPose(CollisionBox c, Pose atPose){
        return c.collides(new CollisionCircle(atPose, getRadius()));
    }

    public boolean collidesAtPose(CollisionBox[] c, Pose atPose){
        for (int i = 0; i < c.length; i++){
            if(c[i].collides(new CollisionCircle(atPose, getRadius()))){
                return true;
            }
        }
        return false;
    }

    public Pose getCenterPoint(){
        return centerPoint;
    }

    public void setCenterPoint(Pose centerPoint){
        this.centerPoint = centerPoint;
    }

    public double getRadius(){
        return radius;
    }
    public void setRadius(double radius){
        this.radius = radius;
    }

    private double distance(Pose point1, Pose point2){
        return Math.sqrt(Math.pow(point2.x() - point1.x(), 2) + Math.pow(point2.y() - point1.y(), 2));
    }

    /**Returns the ID of the CollisionBox that the robot hits along the given path. Returns -1  if it does not collide with anything*/
    public int collisionInPath(CollisionBox[] collisions, Path p){
        p = p.tangent();
        double maxLength = p.curve.length();
        for(int i = 1; i <= (int) Math.ceil(maxLength); i++){
            double t = Math.min(Math.max(i/maxLength, 0), 1);
            for(int j = 0; j < collisions.length; j++){
                if(this.collidesAtPose(collisions[j], p.get(t))){
                    return j;
                }
            }
        }
        return -1;
    }

    /**Returns the Pose at where the robot hits along the given path. Returns null  if it does not collide with anything*/
    public Pose collisionInPathPose(CollisionBox[] collisions, Path p){
        p = p.tangent();
        double maxLength = p.curve.length();
        for(int i = 1; i <= (int) Math.ceil(maxLength); i++){
            double t = Math.min(Math.max(i/maxLength, 0), 1);
            for(int j = 0; j < collisions.length; j++){
                if(this.collidesAtPose(collisions[j], p.get(t))){
                    return p.get(t);
                }
            }
        }
        return null;
    }
}
