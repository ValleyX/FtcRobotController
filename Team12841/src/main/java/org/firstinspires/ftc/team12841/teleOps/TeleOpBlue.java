package org.firstinspires.ftc.team12841.teleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Blue")
public class TeleOpBlue extends TeleOpBase
{
    @Override
    public void init()
    {
        super.pipeline = 1;

        super.init();
    }
}