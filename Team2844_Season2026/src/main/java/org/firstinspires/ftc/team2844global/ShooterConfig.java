package org.firstinspires.ftc.team2844global;

/**
 * This is a Config class for the turrets so that all the values can be stored in one parameter.
 * */
public  class ShooterConfig{
    public final double kP, kI, kD, kS, kV;
    public final String motorName;
    public final String servoName;
    public ShooterConfig(String motorName, String servoName, double kP, double kI, double kD, double kS, double kV){
        this.motorName = motorName;
        this.servoName = servoName;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kS = kS;
        this.kV = kV;
    }

    public ShooterConfig(String motorName, String servoName, PIDF pidf){
        this.motorName = motorName;
        this.servoName = servoName;
        this.kP = pidf.kP;
        this.kI = pidf.kI;
        this.kD = pidf.kD;
        this.kS = pidf.kS;
        this.kV = pidf.kV;
    }
}
