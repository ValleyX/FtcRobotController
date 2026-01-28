package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.MotorExGroup;

//Shooter Subsystem for the turret targeting, and the shooter motor
public class ShooterSubsystem extends SubsystemBase {

    //Shooting Motor
    /**The group of motors that run the flywheel*/
    private MotorExGroup shooterMotors;
    /**The continuous servo that runs the feeder in the neck of the shooter*/
    private CRServo tFeed;

    /**
     * This is the constructor for the shooter Subsystem
     * it sets the motors/servos equal to the object passed in,
     * do so by calling:
     * new Motor(hardwaremap, "____"); for motors,
     * and hardwaremap.get(___.class, "____"); for servos
     * @param shooterMotors The Flywheel motor group that shoots the artifacts
     * @param tFeed The last intaker that is located on the neck of the shooter and determines when the artifact is launched
     */
    public ShooterSubsystem(MotorExGroup shooterMotors, CRServo tFeed){
        this.shooterMotors = shooterMotors;
        this.tFeed = tFeed;
    }

    public void setPower(double power){
        shooterMotors.set(power);
    }

    public void setVelocity(double velocity){
        shooterMotors.setVelocity(velocity);
    }

}
