package org.firstinspires.ftc.team2844simple.autos.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team2844simple.autos.routines.TestSoloRoutine;
import org.firstinspires.ftc.team2844simple.helpers.Constants;
import org.firstinspires.ftc.team2844global.GlobalConstants;

@Autonomous(name = "Simple Blue Solo Test", group = "Blue")
public class BlueTestSolo extends TestSoloRoutine {
    @Override
    public int alliance() {
        return GlobalConstants.BLUE_ALLIANCE;
    }

    @Override
    public int target() {
        return GlobalConstants.BACK_SIDE;
    }
}
