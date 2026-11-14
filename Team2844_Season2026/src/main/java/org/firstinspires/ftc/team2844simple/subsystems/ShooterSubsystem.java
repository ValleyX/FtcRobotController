package org.firstinspires.ftc.team2844simple.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vcs.valleylib.ftc.hardware.FtcSubsystem;
import com.vcs.valleylib.ftc.hardware.Motor;
import com.vcs.valleylib.ftc.hardware.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.team2844simple.helpers.Constants;
import org.firstinspires.ftc.team2844simple.helpers.PIDF;

public class ShooterSubsystem extends FtcSubsystem {
    MotorEx shooter;
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
        shooter.setInverted(false);
        shooter.setRunMode(Motor.RunMode.VelocityControl);
        shooter.setVeloCoefficients(pidf.kP, pidf.kI, pidf.kD);
        shooter.setFeedforwardCoefficients(pidf.kS, pidf.kV);
        shooter.setBuffer(1.0);
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

    public double getCurrent(){
        return shooter.motorEx.getCurrent(CurrentUnit.MILLIAMPS);
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

    /**
     * Sets the shooter velocity.
     * @param rpm Takes a value in rotations per minute
     * */
    public void shoot(double rpm){
        setVelocity(getAsTPS(rpm));
    }

    /**
     * Calculates and sets velocity.
     * Local distance must be updated. [setDistance(...)]
     * Assumes -999 as invalid distance
     * */
    public void shoot(){
        if(distance != Constants.NO_DIS) {
            double vel = getAsTPS(getLinReg());
            setVelocity(vel);
            targetVelocity = vel;
        } else {
            idleShoot();
        }
    }

    public double getLinReg(){
        return Constants.SHOOT_RPM;
    }

    public double getFasterVelocity(double velocity){
        return Constants.SHOOT_RPM+200;
    }

    public double getSlowerVelocity(double velocity){
        return Constants.SHOOT_RPM-200;
    }

    public void idleShoot(){
        setVelocity(getAsTPS(Constants.SHOOTER_IDLE));
    }

    /**
     * Tells you if you are close enough to the given velocity.
     * Assumes valid velocity
     * MEASURES IN TPS, NOT RPM
     * */
    public boolean atVelocity(double tps){
        return ((tps-getAsTPS(Constants.VEL_RPM_THRESHOLD) < getVelocity()) && (getVelocity() < tps+getAsTPS(Constants.VEL_RPM_THRESHOLD)));
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

    public double getPower(){
        return shooter.get();
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
