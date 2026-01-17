package org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Hardwares.LimelightHardware;

@TeleOp(name = "OnePersonDriveBlue")

public class OnePersonDriveBlue extends OnePersonDriveBase{
    @Override
    public void runOpMode() throws InterruptedException {
//        robotHardware = new RobotHardware(this);
//        shooterHardware = new ShooterHardware(this);
        limelightHardware = new LimelightHardware(this);
        limelightHardware.innit(0);
        super.runOpMode();
    }
}
