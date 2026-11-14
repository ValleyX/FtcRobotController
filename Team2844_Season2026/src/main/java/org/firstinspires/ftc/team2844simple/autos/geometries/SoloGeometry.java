package org.firstinspires.ftc.team2844simple.autos.geometries;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.team2844simple.autos.AutoPaths;
import org.firstinspires.ftc.team2844simple.helpers.Constants;

public final class SoloGeometry {
    private static double MID_X = 82.0;
    private static double MID_Y = 112.0;
    private static double MID_DIR = -90.0;
    private SoloGeometry(){}

    /* ----- Poses ----- */
    public static Pose startingPose(PoseFactory f){
        return f.of(72.0+(Constants.ROBOT_LENGTH/2.0), 144-(Constants.ROBOT_WIDTH/2.0), -90.0);}

    public static Pose intakeFromGarden(PoseFactory f){
        return f.of(144-(Constants.ROBOT_LENGTH/2.0),144-(Constants.ROBOT_WIDTH/2.0), 0.0);}

    public static Pose throughMiddle(PoseFactory f){return f.of(MID_X, MID_Y, 90.0);}
    public static Pose flippedShoot(PoseFactory f){return f.of(82.0, 24.0,90.0);}

    /* ----- Paths ----- */
    public static Path toGarden(PoseFactory f){
        return AutoPaths.lineLinearHeading(startingPose(f), intakeFromGarden(f));
    }

    public static Path underpassFromGarden(PoseFactory f){
        Pose start = intakeFromGarden(f);
        Pose middle = throughMiddle(f);
        Pose beforeMiddle = f.of( MID_X- 10*Math.cos(Math.toRadians(MID_DIR)), MID_Y- 25*Math.sin(Math.toRadians(MID_DIR)), MID_DIR);
        Pose afterMiddle =  f.of( MID_X+ 5*Math.cos(Math.toRadians(MID_DIR)),  MID_Y+ 5*Math.sin(Math.toRadians(MID_DIR)), MID_DIR);
        Pose end = flippedShoot(f);
        return Paths.path(
                    Paths.curve(start, beforeMiddle, middle),
                    Paths.curve(middle, afterMiddle, end))
                .heading(Interpolator.piecewise()
                        .until(0.2, Interpolator.constant(start))
                        .until(0.8, Interpolator.linear(start, end))
                        .until(1.0, Interpolator.constant(end)));
    }

    public static Path pickupXFlower(PoseFactory f){
        return Paths.line(flippedShoot(f), AutoPaths.xFlower(f)).linear(flippedShoot(f), AutoPaths.xFlower(f), 0.6);
    }

    public static Path shootAgain(PoseFactory f){
        Pose start = AutoPaths.xFlower(f);
        Pose end = flippedShoot(f);
        return Paths.line(start, end)
                .heading(Interpolator.piecewise()
                        .until(0.3, Interpolator.constant(start))
                        .until(1.0, Interpolator.linear(start, end)));
    }
}
