package org.firstinspires.ftc.team2844.helpers;

public class PIDF {
    public double kP, kI, kD, kS, kV;

    public PIDF(double p,double  i,double  d){
        kP = p;
        kI = i;
        kD = d;
    }

    public PIDF(double p, double i,double d,double s,double v){
        kP = p;
        kI = i;
        kD = d;
        kS = s;
        kV = v;
    }
}
