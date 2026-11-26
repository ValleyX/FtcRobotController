package org.firstinspires.ftc.team2844global;


public class MatchInfo {
    /* ---------- TARGET DATA ---------- */
    public static double targetX = GlobalConstants.NO_POS;
    public static double targetY = GlobalConstants.NO_POS;

    // ?----- alliance tracker -----
    public static int alliance = GlobalConstants.NO_ALLIANCE;
    public static void changeAlliance(int team){
        alliance = team;
    }

    // ?----- side ids -----
    public static int side = GlobalConstants.NO_TARGET;

    public static void changeSide(int newSide){
        side = newSide;
        setTarget();
    }

    public static void setTarget(){
        changeTarget(side+alliance);
    }

    public static int target = GlobalConstants.NO_TARGET;

    /**
     * This method changes the values for the goal's field coordinates depending on case given
     * The field coordinates are rough estimates of where the cell is above when tipped up.
     * @param t 0- Blue Audience 1- Blue Back 2- Red Audience 3- Red Back
     *
     */
    public static void changeTarget(int t) {
        target = t;
        switch(target) {
            case(GlobalConstants.BLUE_AUDIENCE): //Blue Audience
                targetX = GlobalConstants.BLUE_AUDIENCE_X;
                targetY = GlobalConstants.BLUE_AUDIENCE_Y;
                break;
            case(GlobalConstants.BLUE_BACK): //Blue Back
                targetX = GlobalConstants.BLUE_BACK_X;
                targetY = GlobalConstants.BLUE_BACK_Y;
                break;
            case(GlobalConstants.RED_AUDIENCE): //Red Audience
                targetX = GlobalConstants.RED_AUDIENCE_X;
                targetY = GlobalConstants.RED_AUDIENCE_Y;
                break;
            case(GlobalConstants.RED_BACK): //Red Back
                targetX = GlobalConstants.RED_BACK_X;
                targetY = GlobalConstants.RED_BACK_Y;
                break;
            default:
                System.out.println("Target Failed to Change");
                break;
        }
    }
}
