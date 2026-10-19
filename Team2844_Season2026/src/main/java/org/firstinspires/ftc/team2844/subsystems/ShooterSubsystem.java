package org.firstinspires.ftc.team2844.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.vcs.valleylib.ftc.hardware.FtcSubsystem;
import com.vcs.valleylib.ftc.hardware.Motor;
import com.vcs.valleylib.ftc.hardware.MotorEx;

import org.firstinspires.ftc.team2844.helpers.Constants;
import org.firstinspires.ftc.team2844.helpers.PIDF;

public class ShooterSubsystem extends FtcSubsystem {
    MotorEx shooter;
    Servo hood;
    public PIDF pidf;
    private double distance;

    public ShooterSubsystem(HardwareMap hardwareMap, Constants.ShooterConfig config){
        super(hardwareMap);

        pidf = new PIDF(config.kP,config.kI,config.kD,config.kS,config.kV );

        shooter = new MotorEx(hardwareMap, config.motorName);
        shooter.setRunMode(Motor.RunMode.VelocityControl);
        shooter.setVeloCoefficients(pidf.kP, pidf.kI, pidf.kD);
        shooter.setFeedforwardCoefficients(pidf.kS, pidf.kV);

        hood = hardwareMap.get(Servo.class, config.servoName);
    }

    public void setVelocity(double vel){
        shooter.setVelocity(vel);
    }

    public double getVelocity(){
        return shooter.getVelocity();
    }

    /***
     * Uses the distance to find what the expected velocity
     * */
    public double getVelocityLinReg(double distance){
        return distance; //TODO PUT LIN REG HERE
    }
    public double getVelocityLinReg(){
        return distance; //TODO PUT LIN REG HERE
    }

    public void setHood(double pos){
        hood.setPosition(pos);
    }

    public double getHood(){
        return hood.getPosition();
    }

    /***
     * Uses the distance to find what the expected hood pos
     * */
    public double getHoodLinReg(double distance){
        return distance; //TODO PUT LIN REG HERE
    }
    public double getHoodLinReg(){
        return distance; //TODO PUT LIN REG HERE
    }


    /**
     * Calculates and sets lin reg for velocity and hood position.
     * Assumes -999 as invalid velocity and position
     * */
    public void shoot(double velocity, double position){
        double pos = (0.0<position && position < 1.0) ? position : Constants.HOOD_DEFAULT; // if pos is valid, use it; otherwise, use default
        setVelocity(velocity);
        setHood(pos);
    }

    /**
     * Calculates and sets lin reg for velocity and hood position.
     * Assumes -999 as invalid distance
     * */
    public void shoot(double distance){
        //if distance is valid then get the lin regs, otherwise use the idles
        double vel = (distance != Constants.NO_DIS) ? getVelocityLinReg(distance) : Constants.SHOOTER_IDLE;
        double pos = (distance != Constants.NO_DIS) ? getHoodLinReg(distance) : Constants.HOOD_DEFAULT;
        setVelocity(vel);
        setHood(pos);
    }

    /**
     * Calculates and sets lin reg for velocity and hood position.
     * Local distance must be updated. [setDistance(...)]
     * Assumes -999 as invalid distance
     * */
    public void shoot(){
        //if distance is valid then get the lin regs, otherwise use the idles
        double vel = (distance != Constants.NO_DIS) ? getVelocityLinReg() : Constants.SHOOTER_IDLE;
        double pos = (distance != Constants.NO_DIS) ? getHoodLinReg() : Constants.HOOD_DEFAULT;
        setVelocity(vel);
        setHood(pos);
    }

    public void idleShoot(){
        setVelocity(Constants.SHOOTER_IDLE);
        setHood(Constants.HOOD_DEFAULT);
    }

    /**
     * Tells you if you are close enough to the given velocity.
     * Assumes valid velocity
     * */
    public boolean atVelocity(double velocity){
        return ((velocity-Constants.VEL_THRESHOLD < getVelocity()) && (getVelocity() < velocity+Constants.VEL_THRESHOLD));
    }

    /**
     * Tells you if you are close enough to expected lin reg velocity.
     * Assumes updated and valid distance
     * */
    public boolean atVelocity(){
        return ((getVelocityLinReg(distance)-Constants.VEL_THRESHOLD < getVelocity()) && (getVelocity() < getVelocityLinReg(distance)+Constants.VEL_THRESHOLD));
    }

    /**
     * Updates the local distance variable to allow for methods that don't need distance parameters
     * Must be called every loop
     * */
    public void setDistance(double dis){
        distance = dis;
    }







    public void changePIDF(PIDF pidf){
        this.pidf = pidf;
    }

    public PIDF getPIDF(){
        return pidf;
    }
}
