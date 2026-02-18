package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.SavedVars;

//the thing in the middle of the bot that sorts artifacts
public class SpindexerSubsystem extends SubsystemBase {

    /* ---------- Declarations ---------- */
    /** This is the servo that rotates the turn table (its expensive so be careful) */
    private Servo spindexer;

    //The Color Sensors
    /**The color sensors that are in the spindexer locations,
     * they are named color and then the number of which spot it is,
     * then Bay and the number bay it is in
     * (starting from the spot in front of the intake, going clockwise)
     * */
    private ColorSensor
            color1Bay1, color2Bay1,
            color1Bay2, color2Bay2,
            color1Bay3, color2Bay3;

    private int pos = 0;
    boolean bay1 = false;
    boolean bay2 = false;
    boolean bay3 = false;

    int[] bay1green;
    int[] bay2green;
    int[] bay3green;

    int[] bay1blue;
    int[] bay2blue;
    int[] bay3blue;

    int bay1Color;
    int bay2Color;
    int bay3Color;

     /**
     * This is the constructor for the spindexer Subsystem
     * it sets the motors/servos equal to the object passed in,
     * do so by calling:
     * new Motor(hardwaremap, "____"); for motors,
     * and hardwaremap.get(___.class, "____"); for servos
     * @param hardwareMap This constructor requires the hardwareMap to be passed in to make the color Sensors
      * @param spindexer This is the servo that rotates the turn table (its expensive so be careful)
     */

    public SpindexerSubsystem(HardwareMap hardwareMap, Servo spindexer){
        this.spindexer = spindexer;

        color1Bay1 = hardwareMap.get(ColorSensor.class, Constants.CBUS2);
        color2Bay1 = hardwareMap.get(ColorSensor.class, Constants.CBUS3);
        color1Bay2 = hardwareMap.get(ColorSensor.class, Constants.EBUS2);
        color2Bay2 = hardwareMap.get(ColorSensor.class, Constants.EBUS3);
        color1Bay3 = hardwareMap.get(ColorSensor.class, Constants.EBUS0);
        color2Bay3 = hardwareMap.get(ColorSensor.class, Constants.EBUS1);
    }

    public void runToSlotZero(){
        pos = 0;
        spindexer.setPosition(Constants.SLOT_ZERO);
    }

    public void runToSlotOne(){
        pos = 1;
        spindexer.setPosition(Constants.SLOT_ONE);
    }

    public void runToSlotTwo(){
        pos = 2;
        spindexer.setPosition(Constants.SLOT_TWO);
    }

    public void runToSlot(int desiredSlot){
        if(0 <= desiredSlot && desiredSlot < Constants.SLOT_ARRAY.length) {
            pos = desiredSlot;
            spindexer.setPosition(Constants.SLOT_ARRAY[desiredSlot]);
        } else if (desiredSlot >= Constants.SLOT_ARRAY.length){
            pos = Constants.SLOT_ARRAY.length-1;
            spindexer.setPosition(Constants.SLOT_ARRAY[Constants.SLOT_ARRAY.length-1]);
        } else {
            pos = 0;
            spindexer.setPosition(Constants.SLOT_ARRAY[0]);
        }
    }

    public void runToShootSlot(int desiredSlot){
        if(0 <= desiredSlot && desiredSlot < Constants.SLOT_ARRAY.length) {
            pos = desiredSlot;
            spindexer.setPosition(Constants.SLOT_ARRAY[desiredSlot] - 0.005);
        } else if (desiredSlot >= Constants.SLOT_ARRAY.length){
            pos = Constants.SLOT_ARRAY.length-1;
            spindexer.setPosition(Constants.SLOT_ARRAY[Constants.SLOT_ARRAY.length-1] - 0.005);
        } else {
            pos = 0;
            spindexer.setPosition(Constants.SLOT_ARRAY[0]);
        }
    }

    public int getSlot(){
        return pos;
    }

    private boolean ballInBayOne(boolean raw){
        return (color1Bay1.alpha() > Constants.MIN_ALPHA || color2Bay1.alpha() > Constants.MIN_ALPHA);
    }

    public boolean ballInBayOne(){
        return bay1;
    }

    private boolean ballInBayTwo(boolean raw){
        return (color1Bay2.alpha() > Constants.MIN_ALPHA || color2Bay2.alpha() > Constants.MIN_ALPHA);
    }

    public boolean ballInBayTwo(){
        return bay2;
    }

    private boolean ballInBayThree(boolean raw){
        return (color1Bay3.alpha() > Constants.MIN_ALPHA || color2Bay3.alpha() > Constants.MIN_ALPHA);
    }

    public boolean ballInBayThree(){
        return bay3;
    }

    public double bayOneAlpha(){
        return (color1Bay1.alpha() + color2Bay1.alpha())/2.0;
    }
    public double bayTwoAlpha(){
        return (color1Bay2.alpha() + color2Bay2.alpha())/2.0;
    }
    public double bayThreeAlpha(){
        return (color1Bay3.alpha() + color2Bay3.alpha())/2.0;
    }


    public boolean fullSpindexer(){
        return (ballInBayOne() && ballInBayTwo() && ballInBayThree());
    }

    public boolean empty(){
        return (!ballInBayOne() && !ballInBayTwo() && !ballInBayThree());
    }

    public int bayOneColor(){
        if(ballInBayOne()){
            int[] green = bayOneGreen();
            int[] blue = bayOneBlue();
            if(Math.max(green[0] + green[1], blue[0] + blue[1]) == green[0] + green[1]){
                return Constants.GREEN;
            } else {
                return Constants.PURPLE;
            }
        } else {
            return Constants.UNKNOWN_COLOR;
        }
    }

    public int bayTwoColor(){
        if(ballInBayTwo()){
            int[] green = bayTwoGreen();
            int[] blue = bayTwoBlue();
            if(Math.max(green[0] + green[1], blue[0] + blue[1]) == green[0] + green[1]){
                return Constants.GREEN;
            } else {
                return Constants.PURPLE;
            }
        } else {
            return Constants.UNKNOWN_COLOR;
        }
    }

    public int bayThreeColor(){
        if(ballInBayThree()){
            int[] green = bayThreeGreen();
            int[] blue = bayThreeBlue();
            if(Math.max(green[0] + green[1], blue[0] + blue[1]) == green[0] + green[1]){
                return Constants.GREEN;
            } else {
                return Constants.PURPLE;
            }
        } else {
            return Constants.UNKNOWN_COLOR;
        }
    }

    private int[] bayOneBlue(boolean raw){
        return new int[]{color1Bay1.blue(), color2Bay1.blue()};
    }

    private int[] bayOneGreen(boolean raw){
        return new int[]{color1Bay1.green(), color2Bay1.green()};
    }

    private int[] bayTwoBlue(boolean raw){
        return new int[]{color1Bay2.blue(), color2Bay2.blue()};
    }


    private int[] bayTwoGreen(boolean raw){
        return new int[]{color1Bay2.green(), color2Bay2.green()};
    }

    private int[] bayThreeBlue(boolean raw){
        return new int[]{color1Bay3.blue(), color2Bay3.blue()};
    }


    private int[] bayThreeGreen(boolean raw){
        return new int[]{color1Bay3.green(), color2Bay3.green()};
    }

    //public color methods
    public int[] bayOneBlue(){
        return bay1blue;
    }

    public int[] bayOneGreen(){
        return bay1green;
    }

    public int[] bayTwoBlue(){
        return bay2blue;
    }

    public int[] bayTwoGreen(){
        return bay2green;
    }

    public int[] bayThreeBlue(){
        return bay3blue;
    }

    public int[] bayThreeGreen(){
        return bay3green;
    }



    public double getPosition() {
        return spindexer.getPosition();
    }


    public int getSortPos() {
        int currentPattern;

        if(bay1Color+bay2Color+bay3Color == Constants.SORTABLE){

            if(pos%3 == 2){
                int temp1 = bay1Color;
                int temp2 = bay2Color;
                int temp3 = bay3Color;

                bay1Color = temp3;
                bay2Color = temp1;
                bay3Color = temp2;

            } else if(pos%3 == 1) {
                int temp1 = bay1Color;
                int temp2 = bay2Color;
                int temp3 = bay3Color;

                bay1Color = temp3;
                bay2Color = temp1;
                bay3Color = temp2;
            }

            if(bay1Color == Constants.PURPLE && bay2Color == Constants.PURPLE){
                currentPattern = Constants.PATTERN_PPG;
            } else if(bay1Color == Constants.PURPLE && bay3Color == Constants.PURPLE){
                currentPattern = Constants.PATTERN_PGP;
            } else {
                currentPattern = Constants.PATTERN_GPP;
            }

            if(SavedVars.pattern == Constants.PATTERN_GPP){
                if(currentPattern == Constants.PATTERN_GPP){
                    return 3;
                }else if(currentPattern == Constants.PATTERN_PGP){
                    return 4;
                } else {
                    return 5;
                }
            }else if(SavedVars.pattern == Constants.PATTERN_PGP){
                if(currentPattern == Constants.PATTERN_PGP){
                    return 3;
                }else if(currentPattern == Constants.PATTERN_PPG){
                    return 4;
                } else {
                    return 5;
                }
            } else {
                if(currentPattern == Constants.PATTERN_PPG){
                    return 3;
                }else if(currentPattern == Constants.PATTERN_GPP){
                    return 4;
                } else {
                    return 5;
                }
            }
        } else {
            return 3;
        }
    }



    @Override
    public void periodic() {
        bay1 = ballInBayOne(true);
        bay2 = ballInBayTwo(true);
        bay3 = ballInBayThree(true);

        bay1green = bayOneGreen(true);
        bay1blue = bayOneBlue(true);
        bay2green = bayTwoGreen(true);
        bay2blue = bayTwoBlue(true);
        bay3green = bayThreeGreen(true);
        bay3blue = bayThreeBlue(true);

        bay1Color = bayOneColor();
        bay2Color = bayTwoColor();
        bay3Color = bayThreeColor();
    }
}
