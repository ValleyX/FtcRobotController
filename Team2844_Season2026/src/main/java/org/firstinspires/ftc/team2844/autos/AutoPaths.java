package org.firstinspires.ftc.team2844.autos;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.team2844.helpers.Constants;
import org.firstinspires.ftc.team2844.pedro.PedroConstants;

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
    public static Pose park(PoseFactory f){return f.of( 134 - (Constants.ROBOT_LENGTH/2.0), 24.0, 180.0);}
    public static Pose xFlower(PoseFactory f){return f.of(96.0, (Constants.ROBOT_LENGTH/2.0), -90.0);}
    public static Pose yFlower(PoseFactory f){return f.of(144-(Constants.ROBOT_LENGTH/2.0), 96.0, 0.0);}

    /* ----- Path ----- */
    public static Path curveLinearHeading(Pose start, Pose end){
        return Paths.curve(start, end)
                .linear(start, end);
    }

    private static boolean inMiddle(Pose pose, double alliance){
        return (pose.x() < 96.0 && alliance == Constants.BLUE_ALLIANCE) || (pose.x() > 48.0 && alliance == Constants.RED_ALLIANCE);
    }

    public static Path lineLinearHeading(Pose start, Pose end){
        return Paths.line(start, end)
                .linear(start, end);
    }
}
