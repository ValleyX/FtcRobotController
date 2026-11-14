package org.firstinspires.ftc.team2844simple.teleops.testing.alliances;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team2844simple.helpers.Constants;
import org.firstinspires.ftc.team2844simple.teleops.testing.TestingTeleOp;

@TeleOp(name = "Simple Red Testing Op")
public class RedTestingOp extends TestingTeleOp {
    @Override
    protected int alliance() {
        return Constants.RED_ALLIANCE;
    }

}
