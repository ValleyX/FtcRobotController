package org.firstinspires.ftc.team2844.autos.blue;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team2844.autos.AutoPaths;
import org.firstinspires.ftc.team2844.autos.geometries.UpGeometry;
import org.firstinspires.ftc.team2844.autos.routines.DownRoutine;
import org.firstinspires.ftc.team2844.helpers.Constants;

@Autonomous(name = "Blue Down", group = "Blue")
public class BlueDown extends DownRoutine {
    @Override
    public Pose startingPose() {
        return UpGeometry.startingPose(field());
    }

    @Override
    public int alliance() {
        return Constants.BLUE_ALLIANCE;
    }

    @Override
    public int target() {
        return Constants.AUDIENCE_SIDE;
    }

    @Override
    protected PoseFactory field() {
        return AutoPaths.BLUE;
    }
}
