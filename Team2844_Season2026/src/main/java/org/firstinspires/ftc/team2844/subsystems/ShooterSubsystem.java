package org.firstinspires.ftc.team2844.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vcs.valleylib.ftc.hardware.FtcSubsystem;
import com.vcs.valleylib.ftc.hardware.Motor;
import com.vcs.valleylib.ftc.hardware.MotorEx;

import org.firstinspires.ftc.team2844.helpers.Constants;

public class ShooterSubsystem extends FtcSubsystem {
    MotorEx shooter;
    public double kP, kI, kD, kF;
    public final String motorName;

    public ShooterSubsystem(HardwareMap hardwareMap, Constants.ShooterConfig config){
        super(hardwareMap);
        this.motorName = config.motorName;
        this.kP = config.kP;
        this.kI = config.kI;
        this.kD = config.kD;
        this.kF = config.kF;
    }
}
