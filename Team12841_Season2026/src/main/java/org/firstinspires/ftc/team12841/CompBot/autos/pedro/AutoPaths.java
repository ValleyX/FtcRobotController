package org.firstinspires.ftc.team12841.CompBot.autos.pedro;

import com.pedropathing.api.PoseFactory;

import org.firstinspires.ftc.team12841.CompBot.Constants;

public class AutoPaths {

    public static final PoseFactory BLUE =PoseFactory.degrees();
    public static final PoseFactory RED = PoseFactory.degrees().mirrorX(72.0).mirrorY(72.0);
    public static PoseFactory getField(int alliance){
        return (alliance == Constants.BLUE_ALLIANCE)? BLUE: RED;
    }
}
