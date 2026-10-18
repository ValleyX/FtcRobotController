package org.firstinspires.ftc.team2844.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.vcs.valleylib.ftc.hardware.FtcSubsystem;

import org.firstinspires.ftc.team2844.helpers.Constants;

public class TurretSubsystem extends FtcSubsystem {
    private double heading, x, y;
    private AnalogInput encoder1, encoder2;
    private Servo axon1, axon2;
    public TurretSubsystem(HardwareMap hardwareMap) {
        super(hardwareMap);
        heading = 0;

        //TODO REMEMBER TO PUT THE AXONS IN WITH 1.0 BEING FULLY LEFT AND 0.0 BEING FULLY RIGHT
        axon1 = hardwareMap.get(Servo.class, Constants.CHS4);
        axon2 = hardwareMap.get(Servo.class, Constants.CHS5);
        axon2.setDirection(Servo.Direction.REVERSE);
        encoder1 = hardwareMap.get(AnalogInput.class, Constants.CHDD0);
        encoder2 = hardwareMap.get(AnalogInput.class, Constants.CHDD1);
    }

    /**
     * Returns a value between -180 and 180 based on the reading from the axon's encoders
     * */
    public double getAbsoluteDegrees(){
        //gets the average percent of rotation from the axons and puts it in -180 to 180 range.
        double temp = ((((encoder1.getVoltage()/encoder1.getMaxVoltage()) + (encoder2.getVoltage()/encoder2.getMaxVoltage()))/2.0)*Constants.MAX_SERVO_RANGE);
        temp = normalizeTurretDegrees(temp);
        return temp;
    }

    /**Returns a value between -PI and PI based on the reading from the axon's encoders
     * */
    public double getAbsoluteRadians(){
        //gets the average percent of rotation from the axons and puts it in -180 to 180 range.
        double temp = ((((encoder1.getVoltage()/encoder1.getMaxVoltage()) + (encoder2.getVoltage()/encoder2.getMaxVoltage()))/2.0)*Constants.MAX_SERVO_RANGE);
        temp = normalizeTurretDegrees(temp);
        return Math.toRadians(temp);
    }

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

    /**Sets the turret to a field centric degree based on pedro pathings */
    public void setFieldCentricDegrees(double degrees){
        axon1.setPosition(convertDegreesToServo(getFieldCentricDegrees(degrees)));
        axon2.setPosition(convertDegreesToServo(getFieldCentricDegrees(degrees)));
    }

    public double getFieldCentricDegrees(double degrees){
        double temp = Math.max(Math.min(degrees, 180), -180);
        temp -= heading;
        temp = normalizeTurretDegrees(temp);
        return temp;
    }

    public void setFieldCentricRadians(double radians){
        axon1.setPosition(convertRadiansToServo(getFieldCentricRadians(radians)));
        axon2.setPosition(convertRadiansToServo(getFieldCentricRadians(radians)));
    }

    public double getFieldCentricRadians(double radians){
        return Math.toRadians(getFieldCentricDegrees(Math.toDegrees(radians)));
    }


    public double getTargetDegrees(){
        return Math.toDegrees(getTargetRadians());
    }

    public double getTargetRadians(){
        double adjacent = distance(x, y, Constants.targetX, y);
        double opposite = distance(x, y, x, Constants.targetY);
        adjacent = (adjacent != 0.0)? adjacent: 0.1;

        return Math.atan2(opposite, adjacent);
    }

    public void aimToTarget(){
        setFieldCentricRadians(getTargetRadians());
    }


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
        //replace Constants.MAX_SERVO_RANGE with 360.0 if we want to scale the turret down instead of just having a deadspot
        double temp = (Math.max(Math.min(degrees, 180.0), -180.0) + 180.0) / Constants.MAX_SERVO_RANGE;
        return Math.max(Math.min(temp, 1.0), 0.0);
    }

    private double normalizeTurretDegrees(double degrees){
        double temp = degrees;
        while (temp<-180) temp+=180;
        while (temp>180) temp-=180;
        return temp;
    }

    private double distance(double x1, double y1, double x2, double y2){
        return Math.sqrt(Math.pow((x2 -x1), 2) + Math.pow((y2 - y1), 2));
    }

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
}
