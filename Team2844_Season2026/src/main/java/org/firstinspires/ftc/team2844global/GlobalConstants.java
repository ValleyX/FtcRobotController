package org.firstinspires.ftc.team2844global;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.team2844global.collision.CollisionBox;

@Config
@Configurable
public class GlobalConstants {
    /* ---------- No ---------- */
    public static final double NO_TIME = -1.0;

    /* ---------- MATH CONSTANTS ---------- */
    /** Multiply to turn radians per second into rotations per minute, divide to get the opposite*/
    public static final double RAD_PS_TO_RPM = 30.0 / Math.PI;
    /** Multiply to turn meters to inches, and divide to get inches to meters*/
    public static final double METERS_TO_INCHES = 39.3701;


    /* ---------- FIELD CONSTANTS ----------- */
    public static final double CELL_HEIGHT = 53.5;

    public static double BLUE_AUDIENCE_X = 84.75;
    public static double BLUE_AUDIENCE_Y = 53.0;
    public static double BLUE_BACK_X = 84.75;
    public static double BLUE_BACK_Y = 91.0;
    public static double RED_AUDIENCE_X = 59.25;
    public static double RED_AUDIENCE_Y = 53.0;
    public static double RED_BACK_X = 59.25;
    public static double RED_BACK_Y = 91.0;


    /* ---------- ID CONSTANTS ---------- */
    public static final int BLUE_AUDIENCE = 0;
    public static final int BLUE_BACK = 1;
    public static final int RED_AUDIENCE = 2;
    public static final int RED_BACK = 3;

    //? Object IDs
    public static final int POLLEN = 1;
    public static final int BLUE_NECTAR = 2;
    public static final int RED_NECTAR = 3;

    public static final int BLUE_ALLIANCE = 0;
    public static final int RED_ALLIANCE = 2;

    public static final int AUDIENCE_SIDE = 0;
    public static final int BACK_SIDE = 1;

    /* ---------- MISSING INFO ---------- */
    public static final double NO_HEADING = -999;
    public static final double NO_POS = -999;
    public static final double NO_DIS = -999;
    public static final double NO_LL = -999;
    public static final int NO_TARGET = -1;
    public static final int NO_ALLIANCE = -1;
    public static final double TOO_SLOW = -999;
    public static final double TOO_FAST = 999;

    /* ---------- COLLISIONS ---------- */

    public static final int DEFAULT_DEPTH = 8;

    // Corners go counterclockwise.
    public static final CollisionBox BLUE_HIVE_COLLISION = new CollisionBox(
            new Pose(94.73, 52.525),
            new Pose(96.73, 52.525),
            new Pose(96.73, 91.475),
            new Pose(94.73, 91.475)
    );

    public static final CollisionBox RED_HIVE_COLLISION = new CollisionBox(
            new Pose(47.27, 52.525),
            new Pose(49.27, 52.525),
            new Pose(49.27, 91.475),
            new Pose(47.27, 91.475)
    );

    //blue y flower: blue alliance wall, centered at y = 96
    public static final CollisionBox BLUE_YFLOWER_COLLISION = new CollisionBox(
            new Pose(139.0, 93.0),
            new Pose(144.0, 93.0),
            new Pose(144.0, 99.0),
            new Pose(139.0, 99.0)
    );

    //blue x flower: audience wall, centered at x = 96
    public static final CollisionBox BLUE_XFLOWER_COLLISION = new CollisionBox(
            new Pose(93.0, 0.0),
            new Pose(99.0, 0.0),
            new Pose(99.0, 5.0),
            new Pose(93.0, 5.0)
    );

    //red y flower: red alliance wall, centered at y = 48
    public static final CollisionBox RED_YFLOWER_COLLISION = new CollisionBox(
            new Pose(0.0, 45.0),
            new Pose(5.0, 45.0),
            new Pose(5.0, 51.0),
            new Pose(0.0, 51.0)
    );

    //red x flower: back wall, centered at x = 48
    public static final CollisionBox RED_XFLOWER_COLLISION = new CollisionBox(
            new Pose(45.0, 139.0),
            new Pose(51.0, 139.0),
            new Pose(51.0, 144.0),
            new Pose(45.0, 144.0)
    );

    public static final CollisionBox[] COLLISIONS = new CollisionBox[]{
            BLUE_HIVE_COLLISION,
            BLUE_XFLOWER_COLLISION,
            BLUE_YFLOWER_COLLISION,
            RED_HIVE_COLLISION,
            RED_XFLOWER_COLLISION,
            RED_YFLOWER_COLLISION
    };

    public static final double FIELD_SIZE = 144.0;
    /**How thick each wall collision box is. The boxes sit just outside the field, from 0 outwards.*/
    public static final double WALL_THICKNESS = 2.0;

    //audience wall: along the bottom, y = 0
    public static final CollisionBox AUDIENCE_WALL_COLLISION = new CollisionBox(
            new Pose(-WALL_THICKNESS, -WALL_THICKNESS),
            new Pose(FIELD_SIZE + WALL_THICKNESS, -WALL_THICKNESS),
            new Pose(FIELD_SIZE + WALL_THICKNESS, 0.0),
            new Pose(-WALL_THICKNESS, 0.0)
    );

    //blue alliance wall: along the right, x = 144
    public static final CollisionBox BLUE_WALL_COLLISION = new CollisionBox(
            new Pose(FIELD_SIZE, -WALL_THICKNESS),
            new Pose(FIELD_SIZE + WALL_THICKNESS, -WALL_THICKNESS),
            new Pose(FIELD_SIZE + WALL_THICKNESS, FIELD_SIZE + WALL_THICKNESS),
            new Pose(FIELD_SIZE, FIELD_SIZE + WALL_THICKNESS)
    );

    //back wall: along the top, y = 144
    public static final CollisionBox BACK_WALL_COLLISION = new CollisionBox(
            new Pose(-WALL_THICKNESS, FIELD_SIZE),
            new Pose(FIELD_SIZE + WALL_THICKNESS, FIELD_SIZE),
            new Pose(FIELD_SIZE + WALL_THICKNESS, FIELD_SIZE + WALL_THICKNESS),
            new Pose(-WALL_THICKNESS, FIELD_SIZE + WALL_THICKNESS)
    );

    //red alliance wall: along the left, x = 0
    public static final CollisionBox RED_WALL_COLLISION = new CollisionBox(
            new Pose(-WALL_THICKNESS, -WALL_THICKNESS),
            new Pose(0.0, -WALL_THICKNESS),
            new Pose(0.0, FIELD_SIZE + WALL_THICKNESS),
            new Pose(-WALL_THICKNESS, FIELD_SIZE + WALL_THICKNESS)
    );

    public static final CollisionBox[] WALL_COLLISIONS = new CollisionBox[]{
            AUDIENCE_WALL_COLLISION,
            BLUE_WALL_COLLISION,
            BACK_WALL_COLLISION,
            RED_WALL_COLLISION
    };

    /**Every obstacle and every wall: COLLISIONS first, then WALL_COLLISIONS.*/
    public static final CollisionBox[] ALL_COLLISIONS = combine(COLLISIONS, WALL_COLLISIONS);

    private static CollisionBox[] combine(CollisionBox[] first, CollisionBox[] second){
        CollisionBox[] all = new CollisionBox[first.length + second.length];
        System.arraycopy(first, 0, all, 0, first.length);
        System.arraycopy(second, 0, all, first.length, second.length);
        return all;
    }

    /* COLORS */
    public static final double OFF = 0.00;
    public static final double RED = 0.28;
    public static final double ORANGE = 0.33;
    public static final double YELLOW = 0.39;
    public static final double SAGE = 0.44;
    public static final double GREEN = 0.50;
    public static final double AZURE = 0.56;
    public static final double BLUE = 0.61;
    public static final double INDIGO = 0.67;
    public static final double VIOLET = 0.72;
    public static final double WHITE = 1.00;
}
