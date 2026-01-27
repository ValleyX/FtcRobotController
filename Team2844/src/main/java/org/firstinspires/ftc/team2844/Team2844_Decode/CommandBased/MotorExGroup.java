package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.List;

public class MotorExGroup{

    List<MotorEx> motorExList;

    public MotorExGroup(List<MotorEx> motorExList){
        this.motorExList = motorExList;
    }

    /**Sets velocity to the motor in ticks per second*/
    public void setVelocity(double velocity){
        for(int i = 0; i <= motorExList.size(); i++){
            motorExList.get(i).setVelocity(velocity);
        }
    }

    /** Reverses the directions of the motors in the motor groutp*/
    public void setInverted(){
        for(int i = 0; i <= motorExList.size(); i++){
            motorExList.get(i).setInverted(!motorExList.get(i).getInverted());
        }
    }

    /**Sets the power to the motors; ranges -1 to 1, (0 is stopped)*/
    public void set(double power){
        for(int i = 0; i <= motorExList.size(); i++){
            motorExList.get(i).set(power);
        }
    }

    public void setZeroPowerBehavior(Motor.ZeroPowerBehavior behavior){
        for(int i = 0; i <= motorExList.size(); i++){
            motorExList.get(i).setZeroPowerBehavior(behavior);
        }
    }

    public void setVeloCoefficients(double kp, double ki, double kd){
        for(int i = 0; i <= motorExList.size(); i++){
            motorExList.get(i).setVeloCoefficients(kp, ki, kd);
        }
    }

    public void setFeedforwardCoefficients(double ks, double kv, double ka){
        for(int i = 0; i <= motorExList.size(); i++){
            motorExList.get(i).setFeedforwardCoefficients(ks, kv, ka);
        }
    }

    public void setFeedforwardCoefficients(double ks, double kv){
        for(int i = 0; i <= motorExList.size(); i++){
            motorExList.get(i).setFeedforwardCoefficients(ks, kv);
        }
    }

    public void setRunMode(MotorEx.RunMode runMode){
        for(int i = 0; i <= motorExList.size(); i++){
            motorExList.get(i).setRunMode(runMode);
        }
    }


}
