package org.firstinspires.ftc.team12841.teleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Red")
public class TeleOpRed extends TeleOpBase
{
    @Override
    public void init()
    {
        super.pipeline = 2;

        super.init();
    }
}