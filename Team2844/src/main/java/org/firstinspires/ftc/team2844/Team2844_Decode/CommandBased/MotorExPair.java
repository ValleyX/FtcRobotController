package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import java.util.List;

public class MotorExPair {

    MotorEx motor1;
    MotorEx motor2;

    public MotorExPair(MotorEx motor1, MotorEx motor2){
        this.motor1 = motor1;
        this.motor2 = motor2;
    }

    /**Sets velocity to the motor in ticks per second*/
    public void setVelocity(double velocity){
        motor1.setVelocity(velocity);
        motor2.setVelocity(velocity);
    }

    /** Reverses the directions of the motors in the motor groutp*/
    public void setInverted(){
        motor1.setInverted(!motor1.getInverted());
        motor2.setInverted(!motor2.getInverted());
    }

    /**Sets the power to the motors; ranges -1 to 1, (0 is stopped)*/
    public void set(double power){
        motor1.set(power);
        motor2.set(power);
    }

    public void setZeroPowerBehavior(Motor.ZeroPowerBehavior behavior){
        motor1.setZeroPowerBehavior(behavior);
        motor2.setZeroPowerBehavior(behavior);
    }

    public void setVeloCoefficients(double kp, double ki, double kd){
        motor1.setVeloCoefficients(kp, ki, kd);
        motor2.setVeloCoefficients(kp, ki, kd);
    }

    public void setFeedforwardCoefficients(double ks, double kv, double ka){
        motor1.setFeedforwardCoefficients(ks, kv, ka);
        motor2.setFeedforwardCoefficients(ks, kv, ka);
    }

    public void setFeedforwardCoefficients(double ks, double kv){
        motor1.setFeedforwardCoefficients(ks, kv);
        motor2.setFeedforwardCoefficients(ks, kv);
    }

    public void setRunMode(MotorEx.RunMode runMode){
        motor1.setRunMode(runMode);
        motor2.setRunMode(runMode);
    }


    public double getVelocity(){
        return Math.max(motor1.getVelocity(), motor2.getVelocity());
    }
}
