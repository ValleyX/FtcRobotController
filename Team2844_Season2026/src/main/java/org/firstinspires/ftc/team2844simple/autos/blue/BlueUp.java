package org.firstinspires.ftc.team2844simple.autos.blue;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team2844simple.autos.geometries.UpGeometry;
import org.firstinspires.ftc.team2844simple.autos.routines.UpRoutine;
import org.firstinspires.ftc.team2844simple.helpers.Constants;
import org.firstinspires.ftc.team2844global.GlobalConstants;

@Autonomous(name = "Simple Blue Up", group = "Blue")
public class BlueUp extends UpRoutine {

    @Override
    public Pose startingPose() {
        return UpGeometry.startingPose(field());
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
