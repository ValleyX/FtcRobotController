package org.firstinspires.ftc.team2844.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.vcs.valleylib.ftc.hardware.FtcSubsystem;
import com.vcs.valleylib.ftc.hardware.Motor;
import com.vcs.valleylib.ftc.hardware.MotorEx;

import org.firstinspires.ftc.team2844.helpers.Constants;
import org.firstinspires.ftc.team2844.helpers.PIDF;

public class ShooterSubsystem extends FtcSubsystem {
    MotorEx shooter;
    Servo hood;
    public PIDF pidf;
    private double distance;
    private final String motorName;

    private double targetVelocity;
    public ShooterSubsystem(HardwareMap hardwareMap, Constants.ShooterConfig config){
        super(hardwareMap);

        pidf = new PIDF(config.kP,config.kI,config.kD,config.kS,config.kV );
        this.motorName = config.motorName;

        shooter = new MotorEx(hardwareMap, motorName);
        shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        shooter.setInverted(true);
        shooter.setRunMode(Motor.RunMode.VelocityControl);
        shooter.setVeloCoefficients(pidf.kP, pidf.kI, pidf.kD);
        shooter.setFeedforwardCoefficients(pidf.kS, pidf.kV);

        hood = hardwareMap.get(Servo.class, config.servoName);
        hood.setDirection(Servo.Direction.REVERSE);
    }

    /**
     * Sets the shooter motor to a certain velocity in its possible range.
     * Clips the velocity to the min or max if it is outside the range.
     * @param vel Velocity measured in Ticks Per Second (TPS)
     * */
    public void setVelocity(double vel){
        vel = Math.max(Math.min(vel, 2800), 0.0);
        shooter.setVelocity(vel);
    }

    /**@return the velocity of the shooter in TPS*/
    public double getVelocity(){
        return shooter.getVelocity();
    }

    public double getRPM(){
        return getAsRPM(getVelocity());
    }

    /**@return Current surface speed of the robot*/
    public double getShooterSurfaceSpeed(){
        return getShooterSurfaceSpeed(shooter.getVelocity());
    }

    /**
     * @param velocity Shooter Velocity in TPS
     * @return Surface speed of the shooter
     * */
    public double getShooterSurfaceSpeed(double velocity){
        double RPM = getAsRPM(velocity);
        //convert to radians per second
        double radsPerSec = RPM/Constants.RAD_PS_TO_RPM;
        //get the physical distance that one radian travels
        double inchesPerSec = Constants.SHOOTER_RADIUS*radsPerSec;
        //convert to feet
        return inchesPerSec/12.0;
    }

    /**
     * Returns exit velocity with Feet/Second accounting for efficiency
     * */
    public double getBallVelocity(){
        return getShooterSurfaceSpeed()*Constants.EFFICIENCY;
    }

    /**
     * Returns exit velocity with Feet/Second accounting for efficiency
     * @param velocity the velocity in TPS (shooter.getVelocity() works)
     * */
    public double getBallVelocity(double velocity){
        return getShooterSurfaceSpeed(velocity)*Constants.EFFICIENCY;
    }

    public void setHood(double pos){
        hood.setPosition(Math.max(Math.min(pos, Constants.MAX_HOOD), Constants.MIN_HOOD));
    }

    public void setHoodDegrees(double degrees){
        double temp = degrees;
        temp = Math.min(Math.max(temp, Constants.MIN_HOOD_DEGREE), Constants.MAX_HOOD_DEGREE);
        temp = (Constants.MAX_HOOD_DEGREE - temp)/(Constants.MAX_HOOD_DEGREE - Constants.MIN_HOOD_DEGREE);
        temp *= Constants.MAX_HOOD;

        hood.setPosition(Math.max(Math.min(temp, Constants.MAX_HOOD), Constants.MIN_HOOD));
    }

    public double getHoodDegrees(double pos){
        double percent = pos/Constants.MAX_HOOD;
        return Constants.MAX_HOOD_DEGREE -(percent*(Constants.MAX_HOOD_DEGREE-Constants.MIN_HOOD_DEGREE));
    }

    public double getCurrentHoodDegrees(){
        return getHoodDegrees(hood.getPosition());
    }

    /**
     * Returns the degrees that the hood should be set to to get the ball at the right height with the lowest arc.
     * Takes the initial velocity of the ball, and the positive distance from the bot to the cell
     * Accounts for efficiency modifier on velocity
     * @param initialVelocity Units in Feet/Second
     * @param distance distance from the ground in inches
     * */
    public double getExpectedHoodDegrees(double initialVelocity, double distance){
        //? USES A TARGETING FORMULA TO FIND THE BEST ANGLE FOR THE SHOOTER TO AIM BASED ON THE CURRENT VELOCITY
        //? It is called a Trajectory Point-Match Equation
        //? It uses a constant velocity(which the ball is after it is launched) and decides what the best angle to shoot it at is based on how far and high the target is.
        double height = (Constants.CELL_HEIGHT-Constants.LL_HEIGHT)/12.0;
        //initial velocity of the ball squared.
        double v2 = initialVelocity*initialVelocity;
        double v4 = v2*v2;
        distance = distance/12.0;
        double d2 = distance*distance;
        double g = 32.174;


        double inside = v4 -g*(g*d2 + 2*height*v2);
        if(inside < 0.0) return Constants.TOO_SLOW;
        //the square root part of the quadratic formula
        double sqrt = Math.sqrt(inside);
        //?The top half of the quadratic formula
        double lowArc = v2 - sqrt;
        double highArc = v2 + sqrt;
        //?If the top is negative, that means there is no possible angle and the velocity needs to be increased

        double bottom = g*distance;
        double lowAngle = Math.toDegrees(Math.atan2(lowArc, bottom));
        double highAngle = Math.toDegrees(Math.atan2(highArc, bottom));

        if(Constants.MIN_HOOD_DEGREE < lowAngle && lowAngle < Constants.MAX_HOOD_DEGREE){
            return lowAngle;
        } else if(Constants.MIN_HOOD_DEGREE < highAngle && highAngle < Constants.MAX_HOOD_DEGREE){
            return highAngle;
        } else {
            return (lowAngle > Constants.MAX_HOOD_DEGREE)? Constants.TOO_FAST: Constants.TOO_SLOW;
        }
    }

    public double getExpectedHoodDegrees(double initialVelocity){
        return getExpectedHoodDegrees(initialVelocity, get2DDistance());
    }
    public double getExpectedHoodDegrees(){
        return getExpectedHoodDegrees(getBallVelocity());
    }



    public double getHood(){
        return hood.getPosition();
    }


    /**
     * Calculates and sets velocity and hood position.
     * @param rpm Takes a value in rotations per minute
     * @param degrees Takes a value between Min and Max hood angles (60-90), in degrees.
     * */
    public void shoot(double rpm, double degrees){
        setVelocity(getAsTPS(rpm));
        setHoodDegrees(degrees);
    }

    /**
     * Calculates and sets velocity and hood position.
     * Local distance must be updated. [setDistance(...)]
     * Assumes -999 as invalid distance
     * */
    public void shoot(){
        if(distance != Constants.NO_DIS) {
            double vel = Constants.SHOOT_RPM;
            double hoodDegree = getExpectedHoodDegrees(getBallVelocity(getAsTPS(vel)));
            if (hoodDegree == Constants.TOO_SLOW) {
                vel = getFasterVelocity(vel);
                hoodDegree = getExpectedHoodDegrees(getBallVelocity(getAsTPS(vel)));
            } else if (hoodDegree == Constants.TOO_FAST) {
                vel = getSlowerVelocity(vel);
                hoodDegree = getExpectedHoodDegrees(getBallVelocity(getAsTPS(vel)));
            }
            setVelocity(getAsTPS(vel));
            setHoodDegrees((hoodDegree != Constants.TOO_SLOW && hoodDegree != Constants.TOO_FAST)
                    ? hoodDegree
                    : 0.0);
            targetVelocity = getAsTPS(vel);
        } else {
            idleShoot();
        }
    }

    public double getFasterVelocity(double velocity){
        return Constants.SHOOT_RPM+200;
    }

    public double getSlowerVelocity(double velocity){
        return Constants.SHOOT_RPM-200;
    }

    public void idleShoot(){
        setVelocity(getAsTPS(Constants.SHOOTER_IDLE));
        setHood(Constants.HOOD_DEFAULT);
    }

    /**
     * Tells you if you are close enough to the given velocity.
     * Assumes valid velocity
     * MEASURES IN TPS, NOT RPM
     * */
    public boolean atVelocity(double tps){
        return ((tps-Constants.VEL_THRESHOLD < getVelocity()) && (getVelocity() < tps+Constants.VEL_THRESHOLD));
    }

    /**
     * Tells you if you are close enough to expected lin reg velocity.
     * Assumes updated and valid distance
     * */
    public boolean atVelocity(){
        return atVelocity(targetVelocity);
    }

    /**
     * Updates the local distance variable to allow for methods that don't need distance parameters
     * Must be called every loop
     * @param dis distance from LL to the cell. Including all 3 dimensions
     * */
    public void setDistance(double dis){
        distance = dis;
    }

    public double get2DDistance(){
        return get2DDistance(distance);
    }

    public double get2DDistance(double distance){
        return Math.sqrt(Math.max(0.0, Math.pow(distance,2) - Math.pow(Constants.CELL_HEIGHT-Constants.LL_HEIGHT, 2)));
    }

    /**
     * Takes the ticks per second (motor velocity command), returns RPM
     * */
    private double getAsRPM(double tps){
        return (tps/28.0)*60;
    }

    /**
     * Takes the rotations per minute, returns ticks per second (motor velocity command)
     * */
    private double getAsTPS(double rpm){
        return (rpm/60.0)*28.0;
    }

    @NonNull
    @Override
    public String toString() {
        return motorName;
    }

    public void changePIDF(PIDF pidf){
        this.pidf = pidf;
        shooter.setVeloCoefficients(pidf.kP, pidf.kI, pidf.kD);
        shooter.setFeedforwardCoefficients(pidf.kS, pidf.kV);
    }

    public PIDF getPIDF(){
        return new PIDF(pidf.kP, pidf.kI, pidf.kD, pidf.kS, pidf.kV);
    }
}
