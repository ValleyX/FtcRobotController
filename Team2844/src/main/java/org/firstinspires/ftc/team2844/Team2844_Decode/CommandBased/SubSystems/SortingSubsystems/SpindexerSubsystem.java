package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Constants;

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


     /**
     * This is the constructor for the spindexer Subsystem
     * it sets the motors/servos equal to the object passed in,
     * do so by calling:
     * new Motor(hardwaremap, "____"); for motors,
     * and hardwaremap.get(___.class, "____"); for servos
     * @param opMode This constructor requires the opMode to be passed in to make the color Sensors
      * @param spindexer This is the servo that rotates the turn table (its expensive so be careful)
     */

    public SpindexerSubsystem(OpMode opMode, Servo spindexer){
        this.spindexer = spindexer;

        color1Bay1 = opMode.hardwareMap.get(ColorSensor.class, "color1Bay1");
        color2Bay1 = opMode.hardwareMap.get(ColorSensor.class, "color2Bay1");
        color1Bay2 = opMode.hardwareMap.get(ColorSensor.class, "color1Bay2");
        color2Bay2 = opMode.hardwareMap.get(ColorSensor.class, "color2Bay2");
        color1Bay3 = opMode.hardwareMap.get(ColorSensor.class, "color1Bay3");
        color2Bay3 = opMode.hardwareMap.get(ColorSensor.class, "color2Bay3");
    }

    public void runToBayOne(){
        spindexer.setPosition(Constants.BAY_ONE_POS);
    }

    public void runToBayTwo(){
        spindexer.setPosition(Constants.BAY_TWO_POS);
    }

    public void runToBayThree(){
        spindexer.setPosition(Constants.BAY_THREE_POS);
    }

    public int getBay(){
        if(spindexer.getPosition() == Constants.BAY_ONE_POS){
            return 1;
        } else if (spindexer.getPosition() == Constants.BAY_TWO_POS){
            return 2;
        } else if (spindexer.getPosition() == Constants.BAY_THREE_POS){
            return 3;
        } else {
            return 0;
        }
    }

    public void nextBay(){
        if(getBay() == 1){
            runToBayTwo();
        } else if(getBay() == 2){
            runToBayThree();
        }
    }

    public void previousBay(){
        if(getBay() == 2){
            runToBayOne();
        } else if(getBay() == 3){
            runToBayTwo();
        }
    }

    public boolean ballInBayOne(){
        if((color1Bay1.green() +color1Bay1.blue() >= Constants.MIN_COLOR_SUM) ||
                (color2Bay1.green() + color2Bay1.blue() >= Constants.MIN_COLOR_SUM)){
            return true;
        } else {
            return false;
        }
    }

    public boolean ballInBayTwo(){
        if((color1Bay2.green() + color1Bay2.blue() >= Constants.MIN_COLOR_SUM) ||
                (color2Bay2.green() + color2Bay2.blue() >= Constants.MIN_COLOR_SUM)){
            return true;
        } else {
            return false;
        }
    }

    public boolean ballInBayThree(){
        if((color1Bay3.green() + color1Bay3.blue() >= Constants.MIN_COLOR_SUM) ||
                (color2Bay3.green() + color2Bay3.blue() >= Constants.MIN_COLOR_SUM)){
            return true;
        } else {
            return false;
        }
    }

    public int bayOneColor(){
        if(ballInBayOne()){
            if(Math.max(bayOneGreen()[0] + bayOneGreen()[1], bayOneBlue()[0] + bayOneBlue()[1]) == bayOneGreen()[0] + bayOneGreen()[1]){
                return Constants.GREEN;
            } else {
                return Constants.PURPLE;
            }
        } else {
            return 999;
        }
    }

    public int bayTwoColor(){
        if(ballInBayTwo()){
            if(Math.max(bayTwoGreen()[0] + bayTwoGreen()[1], bayTwoBlue()[0] + bayTwoBlue()[1]) == bayTwoGreen()[0] + bayTwoGreen()[1]){
                return Constants.GREEN;
            } else {
                return Constants.PURPLE;
            }
        } else {
            return 999;
        }
    }

    public int bayThreeColor(){
        if(ballInBayThree()){
            if(Math.max(bayThreeGreen()[0] + bayThreeGreen()[1], bayThreeBlue()[0] + bayThreeBlue()[1]) == bayThreeGreen()[0] + bayThreeGreen()[1]){
                return Constants.GREEN;
            } else {
                return Constants.PURPLE;
            }
        } else {
            return 999;
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
}
