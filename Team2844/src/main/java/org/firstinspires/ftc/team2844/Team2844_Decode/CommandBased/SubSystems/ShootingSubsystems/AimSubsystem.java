package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Constants;

public class AimSubsystem extends SubsystemBase {

    //   SERVO DECLARATIONS
    /**The servo that controls the angle the artifact is shot at*/
    private Servo hoodAim;
    /**The servo that controls the heading of the turret.*/
    private CRServo turretAim;

    private AnalogInput axonIn;

    int turnover;

    double lastLoop = 0.0;

    ElapsedTime elapsedTime;
    double targetDegrees;

    /**
     * The constructor for the AimSubsystem, it sets the motors/servos equal to the object passed in,
     * do so by calling:
     * new Motor(hardwaremap, "____"); for motors,
     * and hardwaremap.get(___.class, "____"); for servos
     * @param hoodAim The servo that controls the angle the artifact is shot at
     * @param turretAim The servo that controls the heading of the turret.
     */
    public AimSubsystem(Servo hoodAim, CRServo turretAim, AnalogInput axonIn){
        this.hoodAim = hoodAim;
        this.turretAim = turretAim;
        this.axonIn = axonIn;
        turnover = 0;
        elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        elapsedTime.reset();
    }

    public void aimTurret(double degrees){
        setPosition(degrees*Constants.SERVO_DEGREE_TO_TURRET_DEGREE);
    }

    /**Adds rotation in Degrees*/
    public void moveTurret(double degrees){
        double rotate = (getAxonValue() + degrees*Constants.SERVO_DEGREE_TO_TURRET_DEGREE);
        if(((Constants.MIN_TURN) <= rotate) && (rotate <= (Constants.MAX_TURN))){
            setPosition(rotate);
        } else if((Constants.MIN_TURN) > rotate) {
            setPosition(Constants.MIN_TURN);
        } else if((Constants.MAX_TURN) < rotate) {
            setPosition(Constants.MAX_TURN);
        }
    }

    public double getAxonValue(){
        return ((axonIn.getVoltage()/axonIn.getMaxVoltage())*360) + turnover*360;
    }

    public double getTurretDegrees(){
        return (((axonIn.getVoltage()/axonIn.getMaxVoltage())*360)+ turnover*360)/Constants.SERVO_DEGREE_TO_TURRET_DEGREE;
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

    public boolean setPosition(double degrees){
        targetDegrees = degrees;
        if(!(Constants.MIN_DEGREE <= degrees && degrees <= Constants.MAX_DEGREE)) {
            if(degrees < Constants.MIN_DEGREE){
                targetDegrees = Constants.MAX_DEGREE;
            } else {
                targetDegrees = Constants.MIN_DEGREE;
            }
        }

        double power;
        if(Math.abs(getTurretDegrees() - targetDegrees) < Constants.GAIN_THRESH) {
            power = Math.abs(getTurretDegrees() - targetDegrees) * Constants.TURRET_GAIN;
        } else {
            power = 1.0;
        }

        if (getTurretDegrees() < (targetDegrees - Constants.TURRET_THRESHHOLD)) {
            turretAim.setPower(Math.min(1.0, power));
        } else if (getTurretDegrees() > (targetDegrees + Constants.TURRET_THRESHHOLD)) {
            turretAim.setPower(Math.max(-1.0, -power));
        } else {
            turretAim.setPower(0.0);
            return true;
        }
        return false;
    }

    @Override
    public void periodic() {
        double thisLoop = getAxonValue();
        if(330 <= thisLoop && lastLoop <= 30){
            turnover--;
        } else if (330 <= lastLoop && thisLoop <= 30){
            turnover++;
        }
        lastLoop = thisLoop;

        setPosition(targetDegrees);
    }
}
