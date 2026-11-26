package org.firstinspires.ftc.team2844.autos.blue;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.team2844.autos.geometries.DownGeometry;
import org.firstinspires.ftc.team2844.autos.routines.DownRoutine;
import org.firstinspires.ftc.team2844.helpers.Constants;
import org.firstinspires.ftc.team2844global.GlobalConstants;

@Autonomous(name = "Blue Down", group = "Blue")
@Disabled
public class BlueDown extends DownRoutine {
    @Override
    public Pose startingPose() {
        return DownGeometry.startingPose(field());
    }

    @Override
    public int alliance() {
        return GlobalConstants.BLUE_ALLIANCE;
    }

    @Override
    public int target() {
        return GlobalConstants.AUDIENCE_SIDE;
    }
}
