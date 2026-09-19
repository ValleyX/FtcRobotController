package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** One-person teleop, blue alliance (Limelight pipeline 0). */
@TeleOp(name = "CB OnePersonDrive Blue", group = "QualBot Command")
public class QualBotTeleOpBlue extends QualBotTeleOpBase {

    @Override
    protected int limelightPipeline() {
        return 0;
    }
}
