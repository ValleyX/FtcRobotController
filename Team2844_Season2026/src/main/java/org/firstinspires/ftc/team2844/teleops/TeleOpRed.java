package org.firstinspires.ftc.team2844.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team2844.helpers.Constants;

@TeleOp(name = "Red")
public class TeleOpRed extends TeleOpBase{

    @Override
    public int limelightPipeline() {
        return Constants.RED_AUDIENCE;
    }
}
