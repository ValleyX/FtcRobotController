package org.firstinspires.ftc.team2844.teleops.testing.alliances;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.team2844.helpers.Constants;
import org.firstinspires.ftc.team2844.teleops.testing.TestingTeleOp;
import org.firstinspires.ftc.team2844global.GlobalConstants;

@TeleOp(name = "Red Testing Op")
@Disabled
public class RedTestingOp extends TestingTeleOp {
    @Override
    protected int alliance() {
        return GlobalConstants.RED_ALLIANCE;
    }

}
