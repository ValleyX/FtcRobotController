package org.firstinspires.ftc.team12841.CompBot.autos.pedro.pointers;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team12841.CompBot.Constants;
import org.firstinspires.ftc.team12841.CompBot.autos.pedro.geometry.UpGeometry;
import org.firstinspires.ftc.team12841.CompBot.autos.pedro.routines.UpRoutine;

@Autonomous(name = "Blue Up", group = "Autos")
public class BlueUp extends UpRoutine {

    @Override
    public int alliance() {
        return 0;
    }

    @Override
    public Pose startingPose() {
        return UpGeometry.startingPose(field());
    }
}
