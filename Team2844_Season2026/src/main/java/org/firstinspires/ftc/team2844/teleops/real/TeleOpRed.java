package org.firstinspires.ftc.team2844.teleops.real;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.team2844.helpers.Constants;
import org.firstinspires.ftc.team2844.teleops.TeleOpBase;
import org.firstinspires.ftc.team2844global.GlobalConstants;
import org.firstinspires.ftc.team2844global.MatchInfo;

@TeleOp(name = "Red")
@Disabled
public class TeleOpRed extends TeleOpBase {
    /**MatchInfo.changeAlliance(alliance) MUST BE CALLED AS THE FIRST LINE OF TELEOP BECAUSE IT AFFECTS SETUP LOGIC*/
    @Override
    public int alliance(){
        return GlobalConstants.RED_ALLIANCE;
    }
}
