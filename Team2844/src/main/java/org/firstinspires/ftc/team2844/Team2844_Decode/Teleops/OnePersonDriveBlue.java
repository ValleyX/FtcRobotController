package org.firstinspires.ftc.team2844.Team2844_Decode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team2844.Team2844_Decode.Hardwares.LimelightHardware;
import org.firstinspires.ftc.team2844.Team2844_Decode.Hardwares.RobotHardware;
import org.firstinspires.ftc.team2844.Team2844_Decode.Hardwares.ShooterHardware;

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
