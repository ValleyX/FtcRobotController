package org.firstinspires.ftc.team2844.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.vcs.valleylib.ftc.hardware.FtcSubsystem;

import org.firstinspires.ftc.team2844.helpers.Constants;

public class TurretSubsystem extends FtcSubsystem {
    private double heading, x, y;
    private double tx;
    private final AnalogInput encoder1, encoder2;
    private final Servo axon1, axon2;
    public TurretSubsystem(HardwareMap hardwareMap) {
        super(hardwareMap);
        heading = 0;
        x = 0;
        y = 0;
        tx = 0;

        //TODO REMEMBER TO PUT THE AXONS IN WITH 1.0 BEING FULLY LEFT AND 0.0 BEING FULLY RIGHT
        axon1 = hardwareMap.get(Servo.class, Constants.CHS4);
        axon2 = hardwareMap.get(Servo.class, Constants.CHS5);
        axon2.setDirection(Servo.Direction.REVERSE);
        encoder1 = hardwareMap.get(AnalogInput.class, Constants.CHDD0);
        //TODO CHECK WHAT ENCODER2 RETURNS WHEN THE AXON IS REVERSED
        encoder2 = hardwareMap.get(AnalogInput.class, Constants.CHDD1);
    }

    /* ----- GET DEGREES ----- */
    //? CURRENT ABSOLUTE DEGREES/RADIANS
    /**
     * Returns a value between -180 and 180 based on the reading from the axon's encoders
     * */
    public double getAbsoluteDegrees(){
        //gets the average percent of rotation from the axons and puts it in -180 to 180 range.
        return ((((encoder1.getVoltage()/encoder1.getMaxVoltage()) + (encoder2.getVoltage()/encoder2.getMaxVoltage()))/2.0)*360.0) - 180.0;
    }

    /**Returns a value between -PI and PI based on the reading from the axon's encoders
     * */
    public double getAbsoluteRadians(){
        return Math.toRadians(getAbsoluteDegrees());
    }


    //?FIELD CENTRIC DEGREES, FIRST IS MOST COMPLEX
    /**Returns the degrees of the turret relative to the field.
     * @param targetDegrees a degree between -180 and 180;
     * @param heading current heading of the robot between -180 and 180
     * @return A degree between -180 and 180 that the turret should be set to in order to match the target field centric degree.
     * */
    public double getFieldCentricDegrees(double targetDegrees, double heading){
        double temp = Math.max(Math.min(targetDegrees, 180), -180);
        temp -= heading;
        temp = normalizeTurretDegrees(temp);
        return temp;
    }
    /**Returns the target degrees of the turret relative to the field.
     * @param degrees a target degree between -180 and 180
     * @return A degree between -180 and 180 that the turret should be set to in order to match the target field centric degree.
     * */
    public double getFieldCentricDegrees(double degrees){
        return getFieldCentricDegrees(degrees, heading);
    }

    //?FIELD CENTRIC RADIANS
    /**Returns the degrees of the turret relative to the field.
     * @param radians a degree between -PI and PI
     * @param heading current heading of the robot between -PI and PI
     * @return A degree between -PI and PI that the turret should be set to in order to match the target field centric degree.
     * */
    public double getFieldCentricRadians(double radians, double heading){
        return Math.toRadians(getFieldCentricDegrees(Math.toDegrees(radians), Math.toDegrees(heading)));
    }
    /**Returns the degrees of the turret relative to the field.
     * @param radians a degree between -PI and PI
     * @return A degree between -PI and PI that the turret should be set to in order to match the target field centric degree.
     * */
    public double getFieldCentricRadians(double radians){
        return Math.toRadians(getFieldCentricDegrees(Math.toDegrees(radians)));
    }

    //?FIELD TARGET DEGREES AND RADIANS
    /**
     * @param x the x pose of the robot from 0-144
     * @param y the y pose of the robot from 0-144
     * @param heading the heading of the robot from -PI to PI
     * @return What the relative heading of the turret should be set to in order to aim to the target.
     * */
    public double getRobotTargetRadians(double x, double y, double heading){
        double adjacent = Constants.targetX - x;
        double opposite = Constants.targetY - y;
        adjacent = (adjacent != 0.0)? adjacent: 0.1;

        double temp = Math.atan2(opposite, adjacent);
        return getFieldCentricRadians(temp, heading);
    }

    /**
     * @param x the x pose of the robot from 0-144
     * @param y the y pose of the robot from 0-144
     * @param heading the heading of the robot from -180 to 180
     * @return What the relative heading of the turret should be set to in order to aim to the target.
     * */
    public double getRobotTargetDegrees(double x, double y, double heading){
        return Math.toDegrees(getRobotTargetRadians(x, y, heading));
    }

    public double getRobotTargetDegrees(){
        return getRobotTargetDegrees(x, y, heading);
    }

    public double getRobotTargetRadians(){
        return getRobotTargetRadians(x, y, heading);
    }


    //?ROBOT RELATIVE TARGET DEGREES
    /**
     * @return What the field centric heading of the turret should be set to in order to aim to the target. Units in radians.
     * */
    public double getFieldTargetRadians(){
        double adjacent = Constants.targetX - x;
        double opposite = Constants.targetY - y;
        adjacent = (adjacent != 0.0)? adjacent: 0.1;

        return Math.atan2(opposite, adjacent);
    }
    /**
     * @return What the field centric heading of the turret should be set to in order to aim to the target. Units in degrees.
     * */
    public double getFieldTargetDegrees(){
        return Math.toDegrees(getFieldTargetRadians());
    }

    /* ----- SET DEGREES ----- */
    /**
     * Accepts values from -180 to 180
     * Sets the turrets relative to the robot heading
     * */
    public void setRelativeDegrees(double degrees){
        axon1.setPosition(convertDegreesToServo(degrees));
        axon2.setPosition(convertDegreesToServo(degrees));
    }

    /**
     * Accepts values from -PI to PI
     * Sets the turrets relative to the robot heading
     * */
    public void setRelativeRadians(double radians){
        axon1.setPosition(convertRadiansToServo(radians));
        axon2.setPosition(convertRadiansToServo(radians));
    }

    /**Sets the turret to a field centric degree based on Pedro pathing pose
     * @param degrees The targeted degree from -180 to 180 (zero facing blue)
     * */
    public void setFieldCentricDegrees(double degrees){
        axon1.setPosition(convertDegreesToServo(getFieldCentricDegrees(degrees)));
        axon2.setPosition(convertDegreesToServo(getFieldCentricDegrees(degrees)));
    }

    /**Sets the turret to a field centric degree based on Pedro pathing pose
     * @param radians The targeted degree from -PI to PI (zero facing blue)
     * */
    public void setFieldCentricRadians(double radians){
        axon1.setPosition(convertRadiansToServo(getFieldCentricRadians(radians)));
        axon2.setPosition(convertRadiansToServo(getFieldCentricRadians(radians)));
    }

    public void aimToTarget(){
        setFieldCentricRadians(getFieldTargetRadians());
    }

    public void aimToTx(){
        if(tx != Constants.NO_LL) {
            setRelativeDegrees(tx);
        }
    }

    /* ----- CHECKERS ----- */
    /**
     * Returns true if the turret is aimed at the AprilTag value or pinpoint value
     * */
    public boolean aimed(){
        return (thresholded(getAbsoluteDegrees(), getFieldTargetDegrees())  ||
                thresholded(getAbsoluteDegrees(), tx));
    }


    /* ----- PRIVATE HELPERS ----- */
    /**Accepts values from -PI to PI but clamps it to the physical limits of the axons
     * Returns a value from 0-1 meant to be fed into the axon servos.
     * @return a value from 0-1 meant to be fed into the axon servos
     * */
    private double convertRadiansToServo(double radians){
        return convertDegreesToServo(Math.toDegrees(radians));
    }
    /**Accepts values from -180 to 180 but clamps it to the physical limits of the axons(more like -180 to 175)
     * Returns a value from 0-1 meant to be fed into the axon servos.
     * @return a value from 0-1 meant to be fed into the axon servos
     * */
    private double convertDegreesToServo(double degrees) {
        double temp = normalizeTurretDegrees(degrees);
        //replace Constants.MAX_SERVO_RANGE with 360.0 if we want to scale the turret down instead of just having a dead-spot
        temp = (Math.max(Math.min(temp, 180.0), -180.0) + 180.0) / Constants.MAX_SERVO_RANGE;
        temp *= Constants.DEGREES_PER_SERVO_PERCENT;
        return Math.max(Math.min(temp, 1.0), 0.0);
    }

    /**
     * Takes any degree and translates it to -180 to 180 degrees.
     * Especially useful for turning 0-360 into -180 to 180
     * */
    private double normalizeTurretDegrees(double degrees){
        double temp = degrees;
        while (temp<-180) temp+=360;
        while (temp>180) temp-=360;
        return temp;
    }

    /**
     * Creates a threshold with Constants.TURRET_THRESHOLD_DEGREES
     * @param currentValue The measured value that you are checking
     * @param desiredValue The value that you would like the currentValue to get close to reaching.
     * */
    private boolean thresholded(double currentValue, double desiredValue){
        return (desiredValue - Constants.TURRET_THRESHOLD_DEGREES < currentValue &&
                currentValue < desiredValue + Constants.TURRET_THRESHOLD_DEGREES);
    }


    /* ----- NEEDED UPDATERS ----- */
    public void setHeading(double degrees){
        heading = degrees;
    }

    public void setX(double x){
        this.x = x;
    }

    public void setY(double y){
        this.y = y;
    }

    public void setPose(double x, double y, double degrees){
        setX(x);
        setY(y);
        setHeading(degrees);
    }

    public void setTx(double tx){
        this.tx = tx;
    }
}
