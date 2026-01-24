package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.TankDriveCmdTeleOp;

@TeleOp (name = "Blue Side Tank: Command Based")
@Disabled
public class TankBlueTeleOp extends TankDriveCmdTeleOp {
    @Override
    public void initialize() {
        super.pipelineNum = 0;
        super.initialize();
    }
}
