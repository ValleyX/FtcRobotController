package org.firstinspires.ftc.team2844simple.autos.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team2844simple.autos.routines.TestSoloRoutine;
import org.firstinspires.ftc.team2844simple.helpers.Constants;

@Autonomous(name = "Simple Blue Solo Test", group = "Blue")
public class BlueTestSolo extends TestSoloRoutine {
    @Override
    public int alliance() {
        return Constants.BLUE_ALLIANCE;
    }

    @Override
    public int target() {
        return Constants.BACK_SIDE;
    }
}
