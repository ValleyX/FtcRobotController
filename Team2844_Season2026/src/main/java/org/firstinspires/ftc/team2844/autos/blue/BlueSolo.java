package org.firstinspires.ftc.team2844.autos.blue;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.team2844.autos.geometries.SoloGeometry;
import org.firstinspires.ftc.team2844.autos.routines.SoloRoutine;
import org.firstinspires.ftc.team2844.helpers.Constants;

@Autonomous(name = "Blue Solo", group = "Blue")
@Disabled
public class BlueSolo extends SoloRoutine {

    @Override
    public Pose startingPose() {
        return SoloGeometry.startingPose(field());
    }

    @Override
    public int alliance() {
        return Constants.BLUE_ALLIANCE;
    }

    @Override
    public int target() {
        return Constants.BACK_SIDE;
    }
}
