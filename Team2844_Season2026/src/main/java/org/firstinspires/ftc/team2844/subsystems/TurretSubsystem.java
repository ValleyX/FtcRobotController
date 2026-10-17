package org.firstinspires.ftc.team2844.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.vcs.valleylib.ftc.hardware.FtcSubsystem;
import com.vcs.valleylib.ftc.hardware.Motor;

import org.firstinspires.ftc.team2844.helpers.Constants;

import java.util.Base64;

public class TurretSubsystem extends FtcSubsystem {
    private double heading;
    private AnalogInput encoder1, encoder2;
    private Servo axon1, axon2;
    public TurretSubsystem(HardwareMap hardwareMap) {
        super(hardwareMap);
        heading = 0;
        axon1 = hardwareMap.get(Servo.class, Constants.CHS4);
        axon2 = hardwareMap.get(Servo.class, Constants.CHS5);
        encoder1 = hardwareMap.get(AnalogInput.class, Constants.CHDD0);
        encoder2 = hardwareMap.get(AnalogInput.class, Constants.CHDD1);
    }



    public void setHeading(double degrees){
        heading = degrees;
    }
}
