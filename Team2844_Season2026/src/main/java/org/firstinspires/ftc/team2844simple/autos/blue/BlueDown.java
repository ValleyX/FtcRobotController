package org.firstinspires.ftc.team2844simple.autos.blue;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team2844simple.autos.geometries.DownGeometry;
import org.firstinspires.ftc.team2844simple.autos.routines.DownRoutine;
import org.firstinspires.ftc.team2844simple.helpers.Constants;

@Autonomous(name = "Simple Blue Down", group = "Blue")
public class BlueDown extends DownRoutine {
    @Override
    public Pose startingPose() {
        return DownGeometry.startingPose(field());
    }

    @Override
    public int alliance() {
        return Constants.BLUE_ALLIANCE;
    }

    @Override
    public int target() {
        return Constants.AUDIENCE_SIDE;
    }
}
