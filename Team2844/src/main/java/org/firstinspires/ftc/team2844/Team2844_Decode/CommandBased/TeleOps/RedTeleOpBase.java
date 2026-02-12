package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.TeleOpBase;

@TeleOp(name = "Zamboni Red TeleOp")
public class RedTeleOpBase extends TeleOpBase {
    @Override
    public void initialize() {
        super.pipelineNum = 1;
        super.initialize();
    }
}
