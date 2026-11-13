package org.firstinspires.ftc.team2844simple.autos.blue;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team2844simple.autos.geometries.UpGeometry;
import org.firstinspires.ftc.team2844simple.autos.routines.UpRoutine;
import org.firstinspires.ftc.team2844simple.helpers.Constants;

@Autonomous(name = "Simple Blue Up", group = "Blue")
public class BlueUp extends UpRoutine {

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
        return Constants.BACK_SIDE;
    }
}
