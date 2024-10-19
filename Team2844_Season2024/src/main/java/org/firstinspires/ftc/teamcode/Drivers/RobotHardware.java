package org.firstinspires.ftc.teamcode.Drivers;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {
    public LinearOpMode OpMode_;
    //Constants
    //lift counts per inch
    public final double LIFT_COUNTS_PER_MOTOR_REV = 28;    //  AndyMark Motor Encoder
    public final double LIFT_DRIVE_GEAR_REDUCTION = 60;     // This is < 1.0 if geared UP
    public final double LIFT_ONE_MOTOR_COUNT = LIFT_COUNTS_PER_MOTOR_REV * LIFT_DRIVE_GEAR_REDUCTION;
    public final double LIFT_Distance_in_one_rev = 1.5 * Math.PI; //in
    public final double LIFT_COUNTS_PER_INCH = LIFT_ONE_MOTOR_COUNT / LIFT_Distance_in_one_rev;

    //hang counts per inch
    public final double HANG_COUNTS_PER_MOTOR_REV = 28;    //  AndyMark Motor Encoder
    public final double HANG_DRIVE_GEAR_REDUCTION = 100;     // This is < 1.0 if geared UP
    public final double HANG_ONE_MOTOR_COUNT = HANG_COUNTS_PER_MOTOR_REV * HANG_DRIVE_GEAR_REDUCTION;
    public final double HANG_Distance_in_one_rev = 1.5 * Math.PI; //in
    public final double HANG_COUNTS_PER_INCH = HANG_ONE_MOTOR_COUNT / HANG_Distance_in_one_rev;

    //subextend counts per inch
    public final double SUBEXTEND_COUNTS_PER_MOTOR_REV = 28;    //  AndyMark Motor Encoder
    public final double SUBEXTEND_DRIVE_GEAR_REDUCTION = 20;     // This is < 1.0 if geared UP
    public final double SUBEXTEND_ONE_MOTOR_COUNT = SUBEXTEND_COUNTS_PER_MOTOR_REV * SUBEXTEND_DRIVE_GEAR_REDUCTION;
    public final double SUBEXTEND_Distance_in_one_rev = 1.5 * Math.PI; //in
    public final double SUBEXTEND_COUNTS_PER_INCH = SUBEXTEND_ONE_MOTOR_COUNT / SUBEXTEND_Distance_in_one_rev;



    //Hardware declaration
    //Motors
    public DcMotor motorFrontLeft;
    public DcMotor motorBackLeft;
    public DcMotor motorFrontRight;
    public DcMotor motorBackRight;
    public DcMotor liftMotor; //motor that runs the lift located on the left
    public DcMotor subExtendMotor; //motor that runs the lift located on the right
    public DcMotor intakeMotor; //motor for intake
    public DcMotor hangMotor; //motor for climber

    //servos
    public Servo bucketServo;
    public Servo intakeServo;

    //sensors
    //color sensor
    public NormalizedColorSensor bucketColorSensor;
    public NormalizedColorSensor belowColorSensor;
    //imu
    public IMU imu;

    RobotHardware(LinearOpMode opMode){
        OpMode_ = opMode;

        //Declare drive motors
        motorFrontLeft = OpMode_.hardwareMap.dcMotor.get("leftFront");
        motorBackLeft = OpMode_.hardwareMap.dcMotor.get("leftBack");
        motorFrontRight = OpMode_.hardwareMap.dcMotor.get("rightFront");
        motorBackRight = OpMode_.hardwareMap.dcMotor.get("rightBack");


        // Reverse left motors
        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        //Reset the encoders
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //makes the motors stop when no power is sent to them
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //declare Lift Motor Stuff
        liftMotor = OpMode_.hardwareMap.dcMotor.get("liftMotor");

        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //declare subExtendMotor Stuff
        subExtendMotor = OpMode_.hardwareMap.dcMotor.get("subExtendMotor");

        subExtendMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        subExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        subExtendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        subExtendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //declare intakeMotor Stuff
        intakeMotor = OpMode_.hardwareMap.dcMotor.get("intakeMotor");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //declare hangMotor Stuff
        hangMotor = OpMode_.hardwareMap.dcMotor.get("hangMotor");

        hangMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //declare bucket servo
        bucketServo = OpMode_.hardwareMap.servo.get("bucketServo");
        //bucketServo.setPosition(BUCKET_CLOSED);//TODO; Add in when we have actual mechanical stuff

        //declare intakeServo
        intakeServo = OpMode_.hardwareMap.servo.get("intakeServo");
       // intakeServo.setPosition(BUCKET_CLOSED);//TODO; Add in when we have actual mechanical stuff

        //initialize IMU
        IMU.Parameters newParameters = new IMU.Parameters(
                //TODO; orientation needs update when we get actual bot
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)
        );
        imu = OpMode_.hardwareMap.get(IMU.class,"imu");
        imu.initialize(newParameters);


    }
}
