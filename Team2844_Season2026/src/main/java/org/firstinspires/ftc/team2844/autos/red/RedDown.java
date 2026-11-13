package org.firstinspires.ftc.team2844.autos.red;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.team2844.autos.geometries.DownGeometry;
import org.firstinspires.ftc.team2844.autos.routines.DownRoutine;
import org.firstinspires.ftc.team2844.helpers.Constants;

@Autonomous(name = "Red Down", group = "Red")
@Disabled
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
