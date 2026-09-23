package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** One-person teleop, red alliance (Limelight pipeline 1). */
@TeleOp(name = "CB OnePersonDrive Red", group = "QualBot Command")
public class QualBotTeleOpRed extends QualBotTeleOpBase {

    @Override
    protected int limelightPipeline() {
        return 1;
    }
}
