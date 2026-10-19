package org.firstinspires.ftc.team2844.subsystems;

import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vcs.valleylib.ftc.hardware.FtcSubsystem;

import org.firstinspires.ftc.team2844.helpers.Constants;


public class LightSubsystem extends FtcSubsystem {

    private final Servo timerLight;
    private final Servo pollenLight;
    private final Servo nectarLight;
    private final ElapsedTime run = new ElapsedTime();
    boolean timer;

    public LightSubsystem(HardwareMap hardwareMap) { // Constructor
        super(hardwareMap);
        timerLight  = hardwareMap.get(Servo.class, Constants.EHS3);
        pollenLight = hardwareMap.get(Servo.class, Constants.EHS4);
        nectarLight = hardwareMap.get(Servo.class, Constants.EHS5);
        timer = false;
    }

    public void setColorPollen(double color) {
        pollenLight.setPosition(color);
    }

    public void setColorNectar(double color) {
        nectarLight.setPosition(color);
    }

    public void startTimerLight() {
        run.reset();
        timer = true;
    }

    @Override
    public void periodic() {
        if(timer) {
            if (run.seconds() > 120) {
                timerLight.setPosition(Constants.OFF);
            } else if (run.seconds() > 115) {
                //Blink Red
            } else if (run.seconds() > 100) {
                timerLight.setPosition(Constants.RED);
            } else if (run.seconds() > 60) {
                timerLight.setPosition(Constants.YELLOW);
            } else {
                timerLight.setPosition(Constants.GREEN);
            }
        }
    }
}
