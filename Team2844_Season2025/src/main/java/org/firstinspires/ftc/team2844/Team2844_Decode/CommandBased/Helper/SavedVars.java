package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper;

public class SavedVars {
    public static double startingX = 0.0;
    public static double startingY = 0.0;
    public static double startingHeading = Constants.NO_HEADING;

    public static int pattern = 0;

    public static void reset(){
        startingX = 0.0;
        startingY = 0.0;
        startingHeading = Constants.NO_HEADING;
        pattern = Constants.PATTERN_GPP;
    }
}
