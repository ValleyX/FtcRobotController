package org.firstinspires.ftc.team2844simple.autos.red;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team2844simple.autos.geometries.DownGeometry;
import org.firstinspires.ftc.team2844simple.autos.routines.DownRoutine;
import org.firstinspires.ftc.team2844simple.helpers.Constants;

@Autonomous(name = "Simple Red Down", group = "Red")
public class RedDown extends DownRoutine {
    @Override
    public Pose startingPose() {
        return DownGeometry.startingPose(field());
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
