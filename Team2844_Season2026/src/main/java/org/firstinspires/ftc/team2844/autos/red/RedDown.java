package org.firstinspires.ftc.team2844.autos.red;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team2844.autos.geometries.UpGeometry;
import org.firstinspires.ftc.team2844.autos.routines.DownRoutine;
import org.firstinspires.ftc.team2844.helpers.Constants;

@Autonomous(name = "Red Down", group = "Red")
public class RedDown extends DownRoutine {
    @Override
    public Pose startingPose() {
        return UpGeometry.startingPose(field());
    }

    @Override
    public int alliance() {
        return Constants.RED_ALLIANCE;
    }

    @Override
    public int target() {
        return Constants.BACK_SIDE;
    }
}
