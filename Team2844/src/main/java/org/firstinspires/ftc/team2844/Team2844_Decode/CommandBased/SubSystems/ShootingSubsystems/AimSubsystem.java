package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Constants;

public class AimSubsystem extends SubsystemBase {

    //   SERVO DECLARATIONS
    /**The servo that controls the angle the artifact is shot at*/
    private Servo hoodAim;
    /**The servo that controls the heading of the turret.*/
    private Servo turretAim;

    /**
     * The constructor for the AimSubsystem, it sets the motors/servos equal to the object passed in,
     * do so by calling:
     * new Motor(hardwaremap, "____"); for motors,
     * and hardwaremap.get(___.class, "____"); for servos
     * @param hoodAim The servo that controls the angle the artifact is shot at
     * @param turretAim The servo that controls the heading of the turret.
     */
    public AimSubsystem(Servo hoodAim, Servo turretAim){
        this.hoodAim = hoodAim;
        this.turretAim = turretAim;
    }

    public void aimTurret(double pos){
        turretAim.setPosition(pos);
    }

    /**Adds rotation in servo decimals*/
    public void moveTurret(double rotation){
        double rotate = (turretAim.getPosition() + rotation);
        if(((Constants.MIN_TURN) <= rotate) && (rotate <= (Constants.MAX_TURN))){
            turretAim.setPosition(turretAim.getPosition() + rotation);
        } else if((Constants.MIN_TURN) > rotate) {
            turretAim.setPosition(Constants.MIN_TURN);
        } else if((Constants.MAX_TURN) < rotate) {
            turretAim.setPosition(Constants.MAX_TURN);
        }
    }

    public double getTurretValue(){
        return turretAim.getPosition();
    }

    public void aimHood(double pos){
        hoodAim.setPosition(pos);
    }

    public void moveHood(double rotation){
        double rotate = (hoodAim.getPosition() + rotation);
        if(((Constants.MIN_TURN) <= rotate) && (rotate <= (Constants.MAX_TURN))){
            hoodAim.setPosition(hoodAim.getPosition() + rotation);
        } else if((Constants.MIN_TURN) > rotate) {
            hoodAim.setPosition(Constants.MIN_TURN);
        } else if((Constants.MAX_TURN) < rotate) {
            hoodAim.setPosition(Constants.MAX_TURN);
        }
    }
}
