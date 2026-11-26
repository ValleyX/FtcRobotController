package org.firstinspires.ftc.team2844simple.helpers;

import org.firstinspires.ftc.team2844global.GlobalConstants;
import org.firstinspires.ftc.team2844global.MatchInfo;

public class SavedVars {
    public static double startingX = GlobalConstants.NO_POS;

    public static double startingY = GlobalConstants.NO_POS;

    public static double startingHeading = GlobalConstants.NO_HEADING;
    public static void reset(){
        startingHeading = GlobalConstants.NO_HEADING;
        startingX = GlobalConstants.NO_POS;
        startingY = GlobalConstants.NO_POS;
        MatchInfo.side = GlobalConstants.NO_TARGET;
    }

    public static boolean validPose(){
        return (validHeading() && validPosition());
    }

    public static boolean validPosition(){
        return (startingX != GlobalConstants.NO_POS && startingY != GlobalConstants.NO_POS);
    }

    public static boolean validHeading(){
        return (startingHeading != GlobalConstants.NO_HEADING);
    }
}
