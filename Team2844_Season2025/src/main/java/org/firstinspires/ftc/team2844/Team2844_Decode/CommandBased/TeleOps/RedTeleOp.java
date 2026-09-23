package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.TeleOpBase;

@TeleOp(name = "Zamboni Red TeleOp")
public class RedTeleOp extends TeleOpBase {
    @Override
    public void initialize() {
        super.pipelineNum = Constants.RED_PIPELINE;
        super.initialize();
    }
}
