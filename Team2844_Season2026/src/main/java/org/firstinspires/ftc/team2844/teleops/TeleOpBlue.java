package org.firstinspires.ftc.team2844.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team2844.helpers.Constants;

@TeleOp(name = "Blue")
public class TeleOpBlue extends TeleOpBase{
    @Override
    public int limelightPipeline() {
        return Constants.BLUE_BACK;
    }
}
