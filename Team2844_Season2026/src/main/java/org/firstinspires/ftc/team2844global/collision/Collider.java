package org.firstinspires.ftc.team2844global.collision;

import com.pedropathing.paths.Path;

public interface Collider {
    boolean collides(CollisionBox c);
    boolean collides(CollisionCircle c);
}
