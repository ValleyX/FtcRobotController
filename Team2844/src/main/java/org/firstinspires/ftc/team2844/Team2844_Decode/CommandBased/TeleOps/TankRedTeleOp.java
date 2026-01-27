package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.TankDriveCmdTeleOp;

@TeleOp(name = "Red Side Drive: Command Based")
public class TankRedTeleOp extends TankDriveCmdTeleOp {
    @Override
    public void initialize() {
        super.pipelineNum = 1;
        super.initialize();
    }
}
