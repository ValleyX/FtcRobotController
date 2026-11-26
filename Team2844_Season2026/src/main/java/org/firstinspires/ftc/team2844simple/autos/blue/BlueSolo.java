package org.firstinspires.ftc.team2844simple.autos.blue;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team2844simple.autos.geometries.SoloGeometry;
import org.firstinspires.ftc.team2844simple.autos.routines.SoloRoutine;
import org.firstinspires.ftc.team2844simple.helpers.Constants;
import org.firstinspires.ftc.team2844global.GlobalConstants;

@Autonomous(name = "Simple Blue Solo", group = "Blue")
public class BlueSolo extends SoloRoutine {

    @Override
    public Pose startingPose() {
        return SoloGeometry.startingPose(field());
    }

    @Override
    public int alliance() {
        return GlobalConstants.BLUE_ALLIANCE;
    }

    @Override
    public int target() {
        return GlobalConstants.BACK_SIDE;
    }
}
