package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;

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
        pos = desiredSlot;
        spindexer.setPosition(Constants.SLOT_ARRAY[desiredSlot]);
    }

    public int getSlot(){
        return pos;
    }

    public void nextSlot(){
        if(getSlot() == 0){
            runToSlotOne();
        } else if(getSlot() == 1){
            runToSlotTwo();
        }
    }

    public void previousSlot(){
        if(getSlot() == 1){
            runToSlotZero();
        } else if(getSlot() == 2){
            runToSlotOne();
        }
    }

    public boolean ballInBayOne(){
        return (color1Bay1.alpha() > Constants.MIN_ALPHA || color2Bay1.alpha() > Constants.MIN_ALPHA);
    }

    public boolean ballInBayTwo(){
        return (color1Bay2.alpha() > Constants.MIN_ALPHA || color2Bay2.alpha() > Constants.MIN_ALPHA);
    }

    public boolean ballInBayThree(){
        return (color1Bay3.alpha() > Constants.MIN_ALPHA || color2Bay3.alpha() > Constants.MIN_ALPHA);
    }

    public boolean fullSpindexer(){
        return (ballInBayOne() && ballInBayTwo() && ballInBayThree());
    }

    public boolean empty(){
        return (!ballInBayOne() && !ballInBayTwo() &&   !ballInBayThree());
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

    public int[] bayOneBlue(){
        return new int[]{color1Bay1.blue(), color2Bay1.blue()};
    }

    public int[] bayOneRed(){
        return new int[]{color1Bay1.red(), color2Bay1.red()};
    }

    public int[] bayOneGreen(){
        return new int[]{color1Bay1.green(), color2Bay1.green()};
    }

    public int[] bayTwoBlue(){
        return new int[]{color1Bay2.blue(), color2Bay2.blue()};
    }

    public int[] bayTwoRed(){
        return new int[]{color1Bay2.red(), color2Bay2.red()};
    }

    public int[] bayTwoGreen(){
        return new int[]{color1Bay2.green(), color2Bay2.green()};
    }

    public int[] bayThreeBlue(){
        return new int[]{color1Bay3.blue(), color2Bay3.blue()};
    }

    public int[] bayThreeRed(){
        return new int[]{color1Bay3.red(), color2Bay3.red()};
    }

    public int[] bayThreeGreen(){
        return new int[]{color1Bay3.green(), color2Bay3.green()};
    }

    public double getPosition() {
        return spindexer.getPosition();
    }
}
