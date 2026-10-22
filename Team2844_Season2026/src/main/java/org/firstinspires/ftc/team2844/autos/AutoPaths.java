package org.firstinspires.ftc.team2844.autos;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.team2844.helpers.Constants;

public final class AutoPaths {
    private AutoPaths(){}




    /* ----- PoseFactories ----- */
    public static final PoseFactory BLUE =PoseFactory.degrees();
    public static final PoseFactory RED = PoseFactory.degrees().mirrorX(72.0).mirrorY(72.0);

    public static PoseFactory getField(){
        return getField(Constants.alliance);
    }
    public static PoseFactory getField(int alliance){
        return (alliance == Constants.BLUE_ALLIANCE)? BLUE: RED;
    }




    /* ----- Pose ----- */
    public static Pose park(PoseFactory f){return f.of( 144 - (Constants.ROBOT_LENGTH/2.0), 24.0, 180.0);}
    public static Pose xFlower(PoseFactory f){return f.of(96.0, (Constants.ROBOT_LENGTH/2.0), 90.0);}
    public static Pose yFlower(PoseFactory f){return f.of(144-(Constants.ROBOT_LENGTH/2.0), 96.0, 0.0);}

    public static Pose normalized(Pose p, int alliance){
        return (alliance == Constants.BLUE_ALLIANCE)
                ? p
                : RED.of(p.x(), p.y(), Math.toDegrees(p.heading()));
    }
    public static Pose normalized(Pose p){
        return normalized(p, Constants.alliance);
    }



    /* ----- Path ----- */
    public static Path curveLinearHeading(Pose start, Pose end){
        return Paths.curve(start, end)
                .linear(start, end);
    }

    public static Path routeToPark(PoseFactory f, Pose from){
        return routeToPark(f, from, Constants.alliance);
    }

    public static Path routeToPark(PoseFactory f, Pose from, int alliance){
        Pose p = normalized(from, alliance);
        if(p.x() < 96.0){
            return Paths.through(from, f.of(84.0, p.y(),90.0), f.of(84.0, 24.0, 0.0), park(f)).linear(from, park(f));
        } else {
            return Paths.through(from, f.of(120.0, p.y(),90.0), f.of(120.0, 24.0, 0.0), park(f)).linear(from, park(f));
        }
    }
}
