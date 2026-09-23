package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.Autos.SpecificAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.Autos.FarSimpleBase;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;

@Autonomous(name = "Red Far Simple")
public class RedFarSimple extends FarSimpleBase {
    @Override
    public void initialize() {
        pipeline = Constants.RED_PIPELINE;
        super.initialize();
    }
}