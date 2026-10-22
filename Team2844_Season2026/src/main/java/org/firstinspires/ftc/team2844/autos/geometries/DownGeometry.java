package org.firstinspires.ftc.team2844.autos.geometries;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.vcs.valleylib.core.command.Command;

import org.firstinspires.ftc.team2844.autos.AutoPaths;
import org.firstinspires.ftc.team2844.commands.drive.TurnToCmd;
import org.firstinspires.ftc.team2844.helpers.Constants;
import org.firstinspires.ftc.team2844.subsystems.DriveSubsystem;

public final class DownGeometry {
    private DownGeometry(){}

    /* ----- Poses ----- */
    public static Pose startingPose(PoseFactory f){return f.of(72.0+(Constants.ROBOT_WIDTH/2.0), (Constants.ROBOT_LENGTH/2.0),-90.0);}
    public static Pose readyPickup(PoseFactory f){return f.of(96.0,12.0,90.0);}
    public static Pose shoot(PoseFactory f){return f.of( 96.0, 12.0, -90.0);}


    /* ----- Paths ----- */
    public static Command turnToFlower(DriveSubsystem drive){
        return new TurnToCmd(drive, Math.toRadians(90.0));
    }

    public static Path readyShoot(PoseFactory f){
        Pose start = startingPose(f);
        Pose end =   shoot(f);
        return Paths.curve(start, end)
                .heading(Interpolator.piecewise()
                        .until(0.2, Interpolator.constant(start))
                        .until(1.0, Interpolator.linear(start, end)));
    }

    public static Path pickupXFlower(PoseFactory f){
        return Paths.line(readyPickup(f), AutoPaths.xFlower(f)).constant(readyPickup(f));
    }

    public static Path backToShoot(PoseFactory f){
        Pose start = AutoPaths.xFlower(f);
        Pose end =   shoot(f);
        return Paths.curve(start, end)
                .heading(Interpolator.piecewise()
                        .until(0.5, Interpolator.constant(start))
                        .until(1.0, Interpolator.linear(start, end)));
    }
}
