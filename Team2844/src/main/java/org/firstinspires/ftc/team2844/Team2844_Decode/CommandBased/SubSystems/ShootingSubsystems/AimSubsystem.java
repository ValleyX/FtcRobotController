package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Constants;

public class AimSubsystem extends SubsystemBase {

    //   SERVO DECLARATIONS
    /**The servo that controls the angle the artifact is shot at*/
    private Servo hoodAim;
    /**The servo that controls the heading of the turret.*/
    private Servo turretAim;

    private IMU turretIMU;
    private GoBildaPinpointDriver pinpoint;

    Motor encoder;

    /**
     * The constructor for the AimSubsystem, it sets the motors/servos equal to the object passed in,
     * do so by calling:
     * new Motor(hardwaremap, "____"); for motors,
     * and hardwaremap.get(___.class, "____"); for servos
     * @param hoodAim The servo that controls the angle the artifact is shot at
     * @param turretAim The servo that controls the heading of the turret.
     */
    public AimSubsystem(Servo hoodAim, Servo turretAim, IMU turretIMU, GoBildaPinpointDriver pinpoint, Motor encoder){
        this.hoodAim = hoodAim;
        this.turretAim = turretAim;
        this.turretIMU = turretIMU;
        this.pinpoint = pinpoint;
        this.encoder = encoder;
    }

    public void aimTurret(double degrees){
        turretAim.setPosition(degrees*Constants.DEGREE_TO_SERVO);
    }

    /**Adds rotation in Degrees*/
    public void moveTurret(double degrees){
        double rotate = (turretAim.getPosition() + degrees*Constants.DEGREE_TO_SERVO);
        if(((Constants.MIN_TURN) <= rotate) && (rotate <= (Constants.MAX_TURN))){
            turretAim.setPosition(rotate);
        } else if((Constants.MIN_TURN) > rotate) {
            turretAim.setPosition(Constants.MIN_TURN);
        } else if((Constants.MAX_TURN) < rotate) {
            turretAim.setPosition(Constants.MAX_TURN);
        }
    }

    public double getTurretValue(){
        return turretAim.getPosition();
    }

    public double getTurretServoDegrees(){
        return turretAim.getPosition()/Constants.DEGREE_TO_SERVO;
    }

    public double getEncoderDegrees(){
        return (encoder.getCurrentPosition()*Constants.DEGREES_PER_TICK) + Constants.ENCODER_OFFSET;
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

    public double getEncoderRate(){
        return encoder.getRate();
    }

    public boolean turretBusy(){
        return (encoder.getRate() != 0.0);
    }

    public void resetIMU(){
        turretIMU.resetYaw();
    }

    public double getHeadingAngles(){
        double heading = ((turretIMU.getRobotYawPitchRollAngles().getYaw() - Constants.TURRET_OFFSET)*-1) + getRobotHeading();
        if(heading < 0){
            heading =+ 360;
        }
        return heading;
    }

    public double getHeadingRadians(){
        double heading = ((turretIMU.getRobotYawPitchRollAngles().getYaw() - Constants.TURRET_OFFSET)*-1) + getRobotHeading();
        if(heading < 0){
            heading =+ 360;
        }
        return Math.toRadians(heading);
    }


    //IMU stuff without Offset

    public double getHeadingAnglesWithoutOffset(){
        return turretIMU.getRobotYawPitchRollAngles().getYaw();
    }

    public double getHeadingRadiansWithoutOffset(){
        return Math.toRadians(turretIMU.getRobotYawPitchRollAngles().getYaw());
    }


    public double getRobotHeading(){
        return pinpoint.getHeading(AngleUnit.DEGREES);
    }

    public double getRobotHeadingRadians(){
        return pinpoint.getHeading(AngleUnit.RADIANS);
    }

}
