package org.firstinspires.ftc.team2844.teleops.testing.alliances;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.team2844.helpers.Constants;
import org.firstinspires.ftc.team2844.teleops.testing.TestingTeleOp;

@TeleOp(name = "Blue Testing Op")
@Disabled
public class BlueTestingOp extends TestingTeleOp {
    @Override
    protected int alliance() {
        return Constants.BLUE_ALLIANCE;
    }
}
