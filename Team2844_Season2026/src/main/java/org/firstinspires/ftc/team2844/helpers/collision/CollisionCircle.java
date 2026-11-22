package org.firstinspires.ftc.team2844.helpers.collision;

import com.pedropathing.math.Pose;

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
}
