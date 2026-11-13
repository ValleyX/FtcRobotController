package org.firstinspires.ftc.team2844.autos.red;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.team2844.autos.geometries.SoloGeometry;
import org.firstinspires.ftc.team2844.autos.routines.SoloRoutine;
import org.firstinspires.ftc.team2844.helpers.Constants;

@Autonomous(name = "Red Solo", group = "Red")
@Disabled
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
