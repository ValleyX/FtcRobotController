package org.firstinspires.ftc.team2844.Team2844_Decode.SummerCamp.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * This is the RobotHardware class, it instantiates all of the motors and has methods to run them.
 * It also will include all of the methods for the motors to run in auto and teleop.
 */

public class RobotHardware {
    OpMode opMode;

    // ----------------- Constants -----------------
    public final double COUNTS_PER_MOTOR_REV = 28;
    public final double DRIVE_GEAR_REDUCTION = 20;
    public final double WHEEL_DIAMETER_INCHES = 4.0;
    public final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // ----------------- Motors -----------------
    private DcMotor lMotor;
    private DcMotor rMotor;

    // ----------------- Class Variables  -----------------
    private double drivePower = 0;
    private double turnPower = 0;


    /**
     * This is the Constructor, it requires an OpMode to be passed in to make the hardware maps for all of the motors
     * @param opMode_ If using a class that extends OpMode or LinearOpMode, just pass in "this"
     */
    public RobotHardware(OpMode opMode_) {
        opMode = opMode_;

        lMotor = opMode.hardwareMap.get(DcMotor.class, "lMotor");
        rMotor = opMode.hardwareMap.get(DcMotor.class, "rMotor");

        lMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //If the robot starts moving backwards or turning when you tell it to move forward, these need to be changed
        //TODO Check if the wheels move the correct direction
        lMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /**
     * A method that sets power to both motors
     * Positive values should move the bot forward, negative values should move the bot backward.
     * @param power accepts a number between -1.0 and 1.0
     */
    public void drive(double power){
        drivePower = power;
        calculatePowers();
    }

    /**
     * A method that sets opposite power to each motors, causing it to turn like a tank.
     * Positive values turn to the right, negative values turn to the left.
     * @param power accepts a number between -1.0 and 1.0
     */
    public void turn(double power){
        turnPower = power;
        calculatePowers();
    }


    /**
     * A method that adds the drive and turn powers so that the robot can turn while driving or drive while turning
     */
    private void calculatePowers(){
        double leftPower = drivePower + turnPower;
        double rightPower = drivePower - turnPower;

        if(Math.abs(leftPower) > 1.0){
            leftPower = leftPower / Math.abs(leftPower);
        }

        if(Math.abs(rightPower) > 1.0){
            rightPower = rightPower / Math.abs(rightPower);
        }

        setMotorPowers(leftPower, rightPower);
    }

    /**
     * A method that sets power to both motors
     * @param left power to set to the left motor accepts a number between -1.0 and 1.0
     * @param right power to set to the right motor accepts a number between -1.0 and 1.0
     */
    public void setMotorPowers(double left, double right){
        lMotor.setPower(left);
        rMotor.setPower(right);
    }

    /**
     * Moves the robot a specified amount of inches at a maximum speed
     * @param inches How far the robot will move, in inches
     * @param speed The power at which the motors will be set [0.0, 1.0]
     */
    public void moveForward(double inches, double speed){
        int lMoveInches = lMotor.getCurrentPosition() + (int)(inches*COUNTS_PER_INCH);
        int rMoveInches = rMotor.getCurrentPosition() + (int)(inches*COUNTS_PER_INCH);

        lMotor.setTargetPosition(lMoveInches);
        rMotor.setTargetPosition(rMoveInches);

        lMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lMotor.setPower(speed);
        rMotor.setPower(speed);
    }
}