package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.Autos.SpecificAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.Autos.FarSimpleBase;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;

@Autonomous(name = "Blue Far Simple")
public class BlueFarSimple extends FarSimpleBase {
    @Override
    public void initialize() {
        pipeline = Constants.BLUE_PIPELINE;
        super.initialize();
    }
}
