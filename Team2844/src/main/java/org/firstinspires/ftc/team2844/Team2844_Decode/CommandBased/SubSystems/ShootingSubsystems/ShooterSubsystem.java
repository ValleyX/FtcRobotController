package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.arcrobotics.ftclib.hardware.motors.CRServo;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.MotorExPair;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

//Shooter Subsystem for the turret targeting, and the shooter motor
public class ShooterSubsystem extends SubsystemBase {

    //Shooting Motor
    /**The group of motors that run the flywheel*/
    private MotorExPair shooterMotors;
    private double vel;

    /**
     * This is the constructor for the shooter Subsystem
     * it sets the motors/servos equal to the object passed in,
     * do so by calling:
     * new Motor(hardwaremap, "____"); for motors,
     * and hardwaremap.get(___.class, "____"); for servos
     * @param shooterMotors The Flywheel motor group that shoots the artifacts
     */
    public ShooterSubsystem(MotorExPair shooterMotors){
        vel = 0.0;
        this.shooterMotors = shooterMotors;
    }

    public void setPower(double power){
        shooterMotors.set(power);
    }

    public void setVelocity(double velocity){
        vel = velocity;
        shooterMotors.setVelocity(velocity);
    }

    public double getVelocity(){return shooterMotors.getVelocity();}

    public boolean inRange(double velocity){return (velocity-Constants.VELOCITY_THRESHHOLD < getVelocity()) && (getVelocity() < velocity+Constants.VELOCITY_THRESHHOLD);}

    public BooleanSupplier inRange(DoubleSupplier velocity, DoubleSupplier currentVelocity){
        return () -> (velocity.getAsDouble()-Constants.VELOCITY_THRESHHOLD < currentVelocity.getAsDouble()) &&
                (currentVelocity.getAsDouble() < velocity.getAsDouble()+Constants.VELOCITY_THRESHHOLD);
        //return () -> true;
    }

    public boolean inRange(){return (vel-Constants.VELOCITY_THRESHHOLD < getVelocity()) && (getVelocity() < vel+Constants.VELOCITY_THRESHHOLD);}
}
