package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

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
        spindexer.setPosition(0.0);
    }

    public void runToBayTwo(){
        spindexer.setPosition(0.39);
    }

    public void runToBayThree(){
        spindexer.setPosition(0.78);
    }
}
