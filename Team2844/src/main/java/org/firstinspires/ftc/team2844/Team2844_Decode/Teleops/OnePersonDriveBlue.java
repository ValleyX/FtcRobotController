package org.firstinspires.ftc.team2844.Team2844_Decode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "OnePersonDriveBlue")

public class OnePersonDriveBlue extends OnePersonDriveBase{
    @Override
    public void runOpMode() throws InterruptedException {
        limelightHardware.innit(1);
        super.runOpMode();
    }
}
