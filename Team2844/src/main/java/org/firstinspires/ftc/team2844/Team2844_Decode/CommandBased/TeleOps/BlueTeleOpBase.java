package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.TeleOpBase;

@TeleOp (name = "Zamboni Blue TeleOp")
public class BlueTeleOpBase extends TeleOpBase {
    @Override
    public void initialize() {
        super.pipelineNum = 0;
        super.initialize();
    }
}
