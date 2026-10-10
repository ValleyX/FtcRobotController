package org.firstinspires.ftc.team2844.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vcs.valleylib.ftc.hardware.FtcSubsystem;
import com.vcs.valleylib.ftc.hardware.Motor;
import com.vcs.valleylib.ftc.hardware.MotorEx;

import org.firstinspires.ftc.team2844.helpers.Constants;
import org.firstinspires.ftc.team2844.helpers.PIDF;

public class ShooterSubsystem extends FtcSubsystem {
    MotorEx shooter;
    public PIDF pidf;
    public final String motorName;

    public ShooterSubsystem(HardwareMap hardwareMap, Constants.ShooterConfig config){
        super(hardwareMap);
        this.motorName = config.motorName;

        pidf = new PIDF(config.kP,config.kI,config.kD,config.kS,config.kV );

        shooter = new MotorEx(hardwareMap, motorName);
        shooter.setRunMode(Motor.RunMode.VelocityControl);
        shooter.setVeloCoefficients(pidf.kP, pidf.kI, pidf.kD);
        shooter.setFeedforwardCoefficients(pidf.kS, pidf.kV);
    }

    public void changePIDF(PIDF pidf){
        this.pidf = pidf;
    }

    public PIDF getPIDF(){
        return pidf;
    }
}
