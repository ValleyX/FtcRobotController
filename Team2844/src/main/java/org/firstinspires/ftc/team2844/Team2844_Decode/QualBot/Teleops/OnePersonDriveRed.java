package org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Hardwares.LimelightHardware;

@TeleOp(name = "OnePersonDriveRed")

public class OnePersonDriveRed extends OnePersonDriveBase{
    @Override
    public void runOpMode() throws InterruptedException {
        limelightHardware = new LimelightHardware(this);
        limelightHardware.innit(1);
        super.runOpMode();
    }
}
