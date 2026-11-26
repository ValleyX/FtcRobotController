package org.firstinspires.ftc.team2844.autos.red;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.team2844.autos.geometries.UpGeometry;
import org.firstinspires.ftc.team2844.autos.routines.UpRoutine;
import org.firstinspires.ftc.team2844.helpers.Constants;
import org.firstinspires.ftc.team2844global.GlobalConstants;

@Autonomous(name = "Red Up", group = "Red")
@Disabled
public class RedUp extends UpRoutine {
    @Override
    public Pose startingPose() {
        return UpGeometry.startingPose(field());
    }

    @Override
    public int alliance() {
        return GlobalConstants.RED_ALLIANCE;
    }

    @Override
    public int target() {
        return GlobalConstants.AUDIENCE_SIDE;
    }
}
