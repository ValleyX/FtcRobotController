package org.firstinspires.ftc.team2844simple.autos.red;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team2844simple.autos.geometries.UpGeometry;
import org.firstinspires.ftc.team2844simple.autos.routines.UpRoutine;
import org.firstinspires.ftc.team2844simple.helpers.Constants;

@Autonomous(name = "Simple Red Up", group = "Red")
public class RedUp extends UpRoutine {
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
        return Constants.AUDIENCE_SIDE;
    }
}
