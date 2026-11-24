package org.firstinspires.ftc.team2844simple.autos;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.team2844simple.helpers.Constants;
import org.firstinspires.ftc.team2844simple.pedro.PedroConstants;

import java.util.function.Supplier;

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
    public static Pose park(PoseFactory f){return f.of( 134 - (Constants.ROBOT_LENGTH/2.0), 47 + (Constants.ROBOT_WIDTH/2.0), 180.0);}
    public static Pose xFlower(PoseFactory f){return f.of(96.0, (Constants.ROBOT_LENGTH/2.0), -90.0);}
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
    public static Path lineLinearHeading(Pose start, Pose end){
        return Paths.line(start, end)
                .linear(start, end);
    }

    public static Path routeToPark(PoseFactory f, Pose from){
        return routeToPark(f, from, Constants.alliance);
    }

    public static Path routeToPark(PoseFactory f, Pose from, int alliance){
        Pose p = from;
        Pose correctX;
        Pose correctY;
        if(inMiddle(from, alliance)){
            correctX = f.of(72.0+(Constants.ROBOT_WIDTH/2.0), 0.0, 0.0).withY(from.y());
            correctY = f.of(84.0, 24.0, 0.0);
            return Paths.path(lineLinearHeading(p, correctX).with(PedroConstants.foresightConfig.maxPathSpeed.at(0.75)),Paths.line(correctX, correctY).constant(correctX), lineLinearHeading(correctY, park(f)));
        } else {
            correctX = f.of(120.0, 0.0, 0.0).withY(from.y());
            correctY = f.of(120.0, 24.0, 0.0);
            return Paths.path(lineLinearHeading(p, correctX).with(PedroConstants.foresightConfig.maxPathSpeed.at(0.75)), Paths.line(correctX, correctY).constant(correctX), lineLinearHeading(correctY, park(f)));
        }
    }

    public static Path routeToPark(PoseFactory f, Supplier<Pose> from, int alliance) {
        return routeToPark(f, from.get(), alliance);
    }

    private static boolean inMiddle(Pose pose, double alliance){
        return (pose.x() < 96.0 && alliance == Constants.BLUE_ALLIANCE) || (pose.x() > 48.0 && alliance == Constants.RED_ALLIANCE);
    }

    private static boolean inMiddle(Pose pose){
        return inMiddle(pose, Constants.alliance);
    }
}
