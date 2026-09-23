package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.TeleOpBase;

@TeleOp (name = "Zamboni Blue TeleOp")
public class BlueTeleOp extends TeleOpBase {
    @Override
    public void initialize() {
        super.pipelineNum = Constants.BLUE_PIPELINE;
        super.initialize();
    }
}
