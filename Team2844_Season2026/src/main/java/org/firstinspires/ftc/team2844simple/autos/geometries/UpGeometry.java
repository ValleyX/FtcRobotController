package org.firstinspires.ftc.team2844simple.autos.geometries;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.team2844simple.autos.AutoPaths;
import org.firstinspires.ftc.team2844simple.helpers.Constants;

public final class UpGeometry {
    private UpGeometry(){}

    /* ----- Poses ----- */
    public static Pose startingPose(PoseFactory f){
        return f.of(72.0+(Constants.ROBOT_LENGTH/2.0), 144-(Constants.ROBOT_WIDTH/2.0), -90.0);}

    public static Pose middleGarden(PoseFactory f){
        return f.of(108.0, 120.0, 0.0);
    }

    public static Pose intakeFromGarden(PoseFactory f){
        return f.of(144-(Constants.ROBOT_LENGTH/2.0),144-(Constants.ROBOT_WIDTH/2.0), 0.0);}

    public static Pose readyToShoot(PoseFactory f){
        return f.of(96.0, 108.0, -90.0);
    }

    /* ----- Paths ----- */
    public static Path toGarden(PoseFactory f){
        return Paths.curve(startingPose(f), middleGarden(f), intakeFromGarden(f))
                .heading(Interpolator.piecewise()
                        .until(0.2, Interpolator.constant(startingPose(f)))
                        .until(0.8, Interpolator.linear(startingPose(f), intakeFromGarden(f)))
                        .until(1.0, Interpolator.constant(intakeFromGarden(f)))
                );
    }

    public static Path readyShoot(PoseFactory f, Pose start){
        Pose end = readyToShoot(f);
        return Paths.line(start, end)
                .heading(Interpolator.piecewise()
                        .until(0.2, Interpolator.constant(start))
                        .until(0.8, Interpolator.linear(start, end))
                        .until(1.0, Interpolator.constant(end)));
    }

    public static Path readyShoot1(PoseFactory f){
        return readyShoot(f, intakeFromGarden(f));
    }

    public static Path readyShoot2(PoseFactory f){
        return readyShoot(f, AutoPaths.yFlower(f));
    }

    public static Path pickupYFlower(PoseFactory f){
        return Paths.line(readyToShoot(f), AutoPaths.yFlower(f)).linear(readyToShoot(f), AutoPaths.yFlower(f), 0.6);
    }
}
