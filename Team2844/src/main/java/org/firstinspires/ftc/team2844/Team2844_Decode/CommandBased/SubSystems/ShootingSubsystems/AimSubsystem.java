package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
//import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.RTPAxon;

public class AimSubsystem extends SubsystemBase {
    /*
    //   SERVO DECLARATIONS
    /**The servo that controls the angle the artifact is shot at
    private Servo hoodAim;
    /**The servo that controls the heading of the turret.
    //private Servo axon;

    //private AnalogInput axonIn;
    private Servo axon; //this is the new turret servo
    //public RTPAxon servo;
    private AnalogInput servoEncoder;
    private double servoPos;
    double targetDegrees;
    double targetDegrees1;
    int turnover;
    double TxLL;

    double lastLoop = 0.0;
    double badLLCount=0;
    boolean servoDone = true;

    // Direction enum for servo
    public enum Direction {
        FORWARD,
        REVERSE
    }
    ElapsedTime elapsedTime;
    private AimSubsystemServo.Direction direction;
    double position;

    /**
     * The constructor for the AimSubsystem, it sets the motors/servos equal to the object passed in,
     * do so by calling:
     * new Motor(hardwaremap, "____"); for motors,
     * and hardwaremap.get(___.class, "____"); for servos
     * @param hoodAim The servo that controls the angle the artifact is shot at
     * @param axon The servo that controls the heading of the turret.
     *
    public AimSubsystem(Servo hoodAim, Servo axon, AnalogInput axonIn){
        this.hoodAim = hoodAim;
        this.axon = axon;
        this.servoEncoder = axonIn;
        turnover = 0;
        elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        elapsedTime.reset();
        direction = AimSubsystemServo.Direction.FORWARD;
        //create control loop for axion servo in CR mode with Analog Feedback
        // servo = new RTPAxon(this.turretAim, this.axonIn);
        //CRServeInit();//update control values
        axon.setDirection(Servo.Direction.REVERSE);
        moveTurret(90);
    }


    public void aimTurret(double degrees){
        if(degrees > Constants.MAX_DEGREE) {
            setPosition(Constants.MAX_DEGREE / 360);
        } else if (degrees < Constants.MIN_DEGREE){
            setPosition(Constants.MIN_DEGREE / 360);
        } else {
            setPosition(degrees / 360);
        }
    }

    //This gets the last position commanded, not actual position.
    public double getServoPos() {
        return axon.getPosition();
    }


    // Get current angle from encoder (in degrees)
    /*public double getEncoderAngle() {
        if (servoEncoder == null) return 0;
        //3.3 is volts of max servo output at 360 degrees
        //return (servoEncoder.getVoltage() / 3.3) * (direction.equals(AimSubsystemServo.Direction.REVERSE) ? -360 : 360);

        //did linear fit for deg vs voltage
        //y = 127.3522*x - 28.74304
        return 360-(servoEncoder.getVoltage()*127.3522 - 28.74304) * (direction.equals(AimSubsystemServo.Direction.REVERSE) ? -1 : 1);
    }

    /**Adds rotation in Degrees
    //this will move the servo a the passed degrees (for manual control} JAE
    public void moveTurret(double degrees){
        //*************JAE ADD
        //
        // the servo pid should handle the degrees to move
        //servo.changeTargetRotation(degrees);

        //need to convert deg to 1:1 servo position


        //get where we need to move the turret to by getting current pos and adding LL offset

        //targetDegrees = degrees;
        //Check in bounds
        /*if(!(Constants.MIN_DEGREE <= degrees && degrees <= Constants.MAX_DEGREE)) {
            if(degrees < Constants.MIN_DEGREE){
                targetDegrees = Constants.MAX_DEGREE;
            } else {
                targetDegrees = Constants.MIN_DEGREE;
            }
        }
        targetDegrees = degrees;
        if (targetDegrees >= Constants.MAX_DEGREE) {
            targetDegrees = Constants.MAX_DEGREE;
        }
        //convert from degrees to servo position (0 to 1 = 0 to 360 deg)
        position = targetDegrees*1/360;
        axon.setPosition(position);
       // servoPos = position;
        //***********JAE ADD END

        /*double rotate = (getAxonValue() + degrees*Constants.SERVO_DEGREE_TO_TURRET_DEGREE);
        if(((Constants.MIN_TURN) <= rotate) && (rotate <= (Constants.MAX_TURN))){
            aimTurret(rotate);
        } else if((Constants.MIN_TURN) > rotate) {
            aimTurret(Constants.MIN_TURN);
        } else if((Constants.MAX_TURN) < rotate) {
            aimTurret(Constants.MAX_TURN);
        }
    }

    //this will set the servo pos based on the lime lite tx offset //JAE
    public void setPosLL (double tx) {

        TxLL = tx;//this is the offset from the limelight

        //this is to check if we are still moving
        if (servoDone == false) {
            if (getEncoderAngle() < targetDegrees1+1.5 || getEncoderAngle() > targetDegrees1-1.5){
                servoDone=true;
            }
            //else {
               // System.out.printf("2488: return");
                return;
           // }
        }

        //check for errors from Lime light
        if (TxLL == Constants.NO_LL) {
            badLLCount++;
           // System.out.printf("2488: bad cnt");
            //dont move
           // targetDegrees = getEncoderAngle();
            //turn to 90 deg if we don't have tracking after a 5x 0.02 sec
            if (badLLCount >= 5) {
                targetDegrees1 = 90;
                badLLCount = 0;
                position = targetDegrees1*1/360;
                Math.max(0, position);
                servoDone=true;
                //turretAim.setPosition(position);
            }
        }
        else { // get TX and move to corrected angle

            badLLCount = 0;
            targetDegrees1 = getEncoderAngle() + TxLL; //get adjusted servo position
//            if (TxLL < 1){ // if Tx from LL is small just return
//                return;
//            }
            position = targetDegrees1*1/360;
            axon.setPosition(Math.max(0, position));
            servoDone=false;

//            while (getEncoderAngle() > targetDegrees1+2 || getEncoderAngle() < targetDegrees1-2){
//                turretAim.setPosition(position);
//            }
        }
       // position = targetDegrees*1/360;
       // turretAim.setPosition(position);

    }

    // Log current state for telemetry/debug JAE
    @SuppressLint("DefaultLocale")
    public String log() {
        return String.format(
                "Current Volts: %.3f\n" +
                        "Encoder Angle: %.2f\n" +
                        "Target Deg: %.2f\n" + "Tx LL: %.2f" + " getservoPos: %.2f\n"+
                        "set servo pos:%.2f"+ " badLLcnt:%.1f"+ " servoDone:%b",
                servoEncoder.getVoltage(),
                getEncoderAngle(),
                targetDegrees1,
                TxLL, getServoPos(),
                position,
                badLLCount,
                servoDone
        );
    }

    public double getServoPosConv() {
        return servoPos;
    }*/

/*
    public double getAxonValue(){
        return ((axonIn.getVoltage()/axonIn.getMaxVoltage())*360);// + turnover*360;
    }

    public double getTurretDegrees(){
        return (((axonIn.getVoltage()/axonIn.getMaxVoltage())*360) )
                //+ turnover*360)
                /Constants.SERVO_DEGREE_TO_TURRET_DEGREE;
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

    public void setPosition(double axonCmd){
        axon.setPosition(axonCmd);
    }

    /*public boolean setPosition(double degrees){
        targetDegrees = degrees;
        if(!(Constants.MIN_DEGREE <= degrees && degrees <= Constants.MAX_DEGREE)) {
            if(degrees < Constants.MIN_DEGREE){
                targetDegrees = Constants.MAX_DEGREE;
            } else {
                targetDegrees = Constants.MIN_DEGREE;
            }
        }
/*
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
        }
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
    }*/


    /* --------------------------- Carter's Subsystem --------------------------- */

    //   SERVO DECLARATIONS
    /**
     * The servo that controls the angle the artifact is shot at
     */
    private Servo hoodAim;
    /**
     * The servo that controls the heading of the turret.
     */
    public Servo axon;

    private AnalogInput axonIn;

    int turnover;

    double lastLoop = 0.0;

    ElapsedTime elapsedTime;

    /**
     * The constructor for the AimSubsystem, it sets the motors/servos equal to the object passed in,
     * do so by calling:
     * new Motor(hardwaremap, "____"); for motors,
     * and hardwaremap.get(___.class, "____"); for servos
     *
     * @param hoodAim The servo that controls the angle the artifact is shot at
     * @param axon    The servo that controls the heading of the turret.
     */
    public AimSubsystem(Servo hoodAim, Servo axon, AnalogInput axonIn) {
        this.hoodAim = hoodAim;
        this.axon = axon;
        this.axonIn = axonIn;
        turnover = 0;
        elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        elapsedTime.reset();
    }

    public void aimTurret(double degrees) {
        if (degrees > Constants.MAX_DEGREE) {
            setPosition(Constants.MAX_DEGREE / 360.0);
        } else if (degrees < Constants.MIN_DEGREE) {
            setPosition(Constants.MIN_DEGREE / 360.0);
        } else {
            setPosition(degrees / 360.0);
        }
    }

    /**
     * Adds rotation in Degrees
     */
    public void moveTurret(double degrees) {
        double rotate = (getAxonValue() + degrees * Constants.SERVO_DEGREE_TO_TURRET_DEGREE);
        if (((Constants.MIN_TURN) <= rotate) && (rotate <= (Constants.MAX_TURN))) {
            aimTurret(rotate);
        } else if ((Constants.MIN_TURN) > rotate) {
            aimTurret(Constants.MIN_TURN);
        } else if ((Constants.MAX_TURN) < rotate) {
            aimTurret(Constants.MAX_TURN);
        }
    }

    public double getAxonValue() {
        return (((axonIn.getVoltage() - Constants.MIN_VOLTAGE) / (Constants.MAX_VOLTAGE - Constants.MIN_VOLTAGE)) * 360.0);// + turnover*360;
    }

    public double getTurretDegrees() {
        return (((axonIn.getVoltage() / axonIn.getMaxVoltage()) * 360.0));
                //+ turnover*360)
                // Constants.SERVO_DEGREE_TO_TURRET_DEGREE;
    }


    public void aimHood(double pos) {
        hoodAim.setPosition(pos);
    }

    public void moveHood(double rotation) {
        double rotate = (hoodAim.getPosition() + rotation);
        if (((Constants.MIN_TURN) <= rotate) && (rotate <= (Constants.MAX_TURN))) {
            hoodAim.setPosition(hoodAim.getPosition() + rotation);
        } else if ((Constants.MIN_TURN) > rotate) {
            hoodAim.setPosition(Constants.MIN_TURN);
        } else if ((Constants.MAX_TURN) < rotate) {
            hoodAim.setPosition(Constants.MAX_TURN);
        }
    }

    /*public boolean setPosition(double degrees){
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
    }*/

    public void setPosition(double axonCmd) {
        axon.setPosition(axonCmd);
    }

    @Override
    public void periodic() {

    }

    public double getVoltage() {
        return axonIn.getVoltage();
    }
}
