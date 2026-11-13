package org.firstinspires.ftc.team2844simple.autos.red;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team2844simple.autos.geometries.SoloGeometry;
import org.firstinspires.ftc.team2844simple.autos.routines.SoloRoutine;
import org.firstinspires.ftc.team2844simple.helpers.Constants;

@Autonomous(name = "Simple Red Solo", group = "Red")
public class RedSolo extends SoloRoutine {

    @Override
    public Pose startingPose() {
        return SoloGeometry.startingPose(field());
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
