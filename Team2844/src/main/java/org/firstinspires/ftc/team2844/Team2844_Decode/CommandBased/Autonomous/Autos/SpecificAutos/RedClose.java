package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.Autos.SpecificAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.Autos.CloseAutoBase;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;

@Autonomous(name = "Red Zamboni Close", group = "Autonomous")
public class RedClose extends CloseAutoBase {
    @Override
    public void initialize() {
        super.pipeline = Constants.RED_PIPELINE;
        Constants.CPICKUP1_Y = Constants.CREDPICKUP1_Y;
        Constants.CPICKUP2_Y = Constants.CREDPICKUP2_Y;
        super.initialize();
    }
}