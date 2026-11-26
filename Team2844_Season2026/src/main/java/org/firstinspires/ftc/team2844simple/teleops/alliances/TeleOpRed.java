package org.firstinspires.ftc.team2844simple.teleops.alliances;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team2844simple.helpers.Constants;
import org.firstinspires.ftc.team2844simple.teleops.TeleOpBase;
import org.firstinspires.ftc.team2844global.GlobalConstants;
import org.firstinspires.ftc.team2844global.MatchInfo;

@TeleOp(name = "Simple Red")
public class TeleOpRed extends TeleOpBase {
    /**MatchInfo.changeAlliance(alliance) MUST BE CALLED AS THE FIRST LINE OF TELEOP BECAUSE IT AFFECTS SETUP LOGIC*/
    @Override
    public int alliance(){
        return GlobalConstants.RED_ALLIANCE;
    }
}
