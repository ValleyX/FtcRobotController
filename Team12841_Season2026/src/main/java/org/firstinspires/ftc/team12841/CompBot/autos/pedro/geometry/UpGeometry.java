package org.firstinspires.ftc.team12841.CompBot.autos.pedro.geometry;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.team12841.CompBot.Constants;

public class UpGeometry {
    private UpGeometry(){}

    public static Pose startingPose(PoseFactory f){
        //This starting pose is against the wall right on the line.
        return f.of(72.0+(Constants.ROBOT_LENGTH/2.0), 144-(Constants.ROBOT_WIDTH/2.0), -90.0);}

    public static Pose parkPose(PoseFactory f){
        return f.of(134.0, 48.0, 180.0);
    }

    public static Pose parkControlPoint(PoseFactory f){
        return f.of(144.0, 130.0, 0.0);
    }

    public static Path getPath(PoseFactory f){
        return Paths.curve(startingPose(f), parkControlPoint(f), parkPose(f)).linear(startingPose(f), parkPose(f));
    }
}
