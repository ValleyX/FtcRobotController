package org.firstinspires.ftc.team2844.helpers.collision;

public interface Collider {
    boolean collides(CollisionBox c);
    boolean collides(CollisionCircle c);
}
