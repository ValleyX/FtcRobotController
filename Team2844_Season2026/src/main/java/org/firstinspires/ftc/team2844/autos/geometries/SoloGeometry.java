package org.firstinspires.ftc.team2844.autos.geometries;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.team2844.autos.AutoPaths;
import org.firstinspires.ftc.team2844.helpers.Constants;

public final class SoloGeometry {
    private SoloGeometry(){}

    /* ----- Poses ----- */
    public static Pose startingPose(PoseFactory f){
        return f.of(72.0+(Constants.ROBOT_LENGTH/2.0), 144-(Constants.ROBOT_WIDTH/2.0), 0.0);}

    public static Pose intakeFromGarden(PoseFactory f){
        return f.of(144-(Constants.ROBOT_LENGTH/2.0),144-(Constants.ROBOT_WIDTH/2.0), 0.0);}

    public static Pose throughMiddle(PoseFactory f){return f.of(82.0, 112.0, 90.0);}
    public static Pose flippedShoot(PoseFactory f){return f.of(82.0, 24.0,45.0);}

    /* ----- Paths ----- */
    public static Path toGarden(PoseFactory f){
        return AutoPaths.curveLinearHeading(startingPose(f), intakeFromGarden(f));
    }

    public static Path underpassFromGarden(PoseFactory f){
        Pose start = intakeFromGarden(f);
        Pose middle = throughMiddle(f);
        Pose end = flippedShoot(f);
        return Paths.through(start, middle, end)
                .heading(Interpolator.piecewise()
                        .until(0.2, Interpolator.constant(start))
                        .until(0.8, Interpolator.linear(start, end))
                        .until(1.0, Interpolator.constant(end)));
    }

    public static Path pickupXFlower(PoseFactory f){
        return Paths.curve(flippedShoot(f), AutoPaths.xFlower(f)).linear(flippedShoot(f), AutoPaths.xFlower(f), 0.6);
    }

    public static Path shootAgain(PoseFactory f){
        Pose start = AutoPaths.xFlower(f);
        Pose end = flippedShoot(f);
        return Paths.curve(start, end)
                .heading(Interpolator.piecewise()
                        .until(0.3, Interpolator.constant(start))
                        .until(1.0, Interpolator.linear(start, end)));
    }
}
