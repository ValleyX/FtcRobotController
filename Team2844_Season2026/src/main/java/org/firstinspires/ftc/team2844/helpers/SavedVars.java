package org.firstinspires.ftc.team2844.helpers;

public class SavedVars {
    public static double startingX = Constants.NO_POS;

    public static double startingY = Constants.NO_POS;

    public static double startingHeading = Constants.NO_HEADING;

    public static void reset(){
        startingHeading = Constants.NO_HEADING;
        startingX = Constants.NO_POS;
        startingY = Constants.NO_POS;
    }
}
