package org.firstinspires.ftc.team12841.Drivers;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class RobotHardware {
    public LinearOpMode OpMode_;
    //Constants
    //lift counts per inch
    public static final double LIFT_COUNTS_PER_MOTOR_REV = 28;    //  AndyMark Motor Encoder
    public static final double LIFT_DRIVE_GEAR_REDUCTION = 60;//60;     // This is < 1.0 if geared UP was 60
    public static final double LIFT_ONE_MOTOR_COUNT = LIFT_COUNTS_PER_MOTOR_REV * LIFT_DRIVE_GEAR_REDUCTION;
    public static final double LIFT_Distance_in_one_rev = 1.5 * Math.PI; //in
    public static final double LIFT_COUNTS_PER_INCH = LIFT_ONE_MOTOR_COUNT / LIFT_Distance_in_one_rev;

    //hang counts per inch
    public static final double HANG_COUNTS_PER_MOTOR_REV = 28;    //  AndyMark Motor Encoder
    public static final double HANG_DRIVE_GEAR_REDUCTION = 81;     // This is < 1.0 if geared UP was 100
    public static final double HANG_ONE_MOTOR_COUNT = HANG_COUNTS_PER_MOTOR_REV * HANG_DRIVE_GEAR_REDUCTION;
    public static final double HANG_Distance_in_one_rev = 1.5 * Math.PI; //in
    public static final double HANG_COUNTS_PER_INCH = HANG_ONE_MOTOR_COUNT / HANG_Distance_in_one_rev;

    //subextend counts per inch
    public static final double SUBEXTEND_COUNTS_PER_MOTOR_REV = 28;    //  AndyMark Motor Encoder
    public static final double SUBEXTEND_DRIVE_GEAR_REDUCTION = 20;     // This is < 1.0 if geared UP
    public static final double SUBEXTEND_ONE_MOTOR_COUNT = SUBEXTEND_COUNTS_PER_MOTOR_REV * SUBEXTEND_DRIVE_GEAR_REDUCTION;
    public static final double SUBEXTEND_Distance_in_one_rev = 1.5 * Math.PI; //in
    public static final double SUBEXTEND_COUNTS_PER_INCH = SUBEXTEND_ONE_MOTOR_COUNT / SUBEXTEND_Distance_in_one_rev;


    //for the servo that grabs the specimen off the wall
    public static final double CLAW_SERVO_DISENGAGED = 1; //1
    public static final double CLAW_SERVO_ENGAGED = .65; //.65

    //subextend servo constants
    public static final double INTAKE_SERVO_INIT = .55;
    public static final double INTAKE_SERVO_UP = 0.25;
    public static final double INTAKE_SERVO_DOWN = 0.425;

    //hang servo constant
    public static final double HANG_SERVO_LATCHED = 0;
    public static final double HANG_SERVO_RELEASED = .05;

    //sample servo constants
    public static final double SAMPLE_SERVO_INTAKE = -1;
    public static final double SAMPLE_SERVO_OUTTAKE = 1;



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
    public Servo clawServo;
    public Servo intakeServo;
    public Servo hangServo;
    public Servo sampleServo; //servo for the subextend grabby thingy. Continous rotation servo


    //sensors
    //color sensor
    public NormalizedColorSensor bucketColorSensor; //i2c 2 control hub
   // public NormalizedColorSensor belowColorSensor;
    //imu
    public IMU imu;
    //touch sensors
    public TouchSensor liftTouch; //senses when lift is all the way down
    public TouchSensor rightBase; //right bump touch sensor for hanging
    public TouchSensor leftBase;//left bump touch sensor for hanging

    public TouchSensor subTouch;//limit button for the sub extend

    //Blinkins
    public RevBlinkinLedDriver blinkinLedDriver; //primary blinkin system

    //public RevBlinkinLedDriver winkinLedDriver; //secondary blinkin system
    public RevBlinkinLedDriver.BlinkinPattern pattern;


    //specific LED pattern declaration
    public RevBlinkinLedDriver.BlinkinPattern autoPattern;
    public RevBlinkinLedDriver.BlinkinPattern redPattern;
    public RevBlinkinLedDriver.BlinkinPattern bluePattern;
    public RevBlinkinLedDriver.BlinkinPattern endgamePattern;


    public RobotHardware(LinearOpMode opMode){
        OpMode_ = opMode;

        //Declare drive motors
        motorFrontLeft = OpMode_.hardwareMap.dcMotor.get("leftFront");
        motorBackLeft = OpMode_.hardwareMap.dcMotor.get("leftRear");
        motorFrontRight = OpMode_.hardwareMap.dcMotor.get("rightFront");
        motorBackRight = OpMode_.hardwareMap.dcMotor.get("rightRear");


        // Reverse left motors
        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        /*//Reset the encoders
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/

        //makes the motors stop when no power is sent to them
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //declare Lift Motor Stuff
        liftMotor = OpMode_.hardwareMap.dcMotor.get("liftMotor");

        //liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        //set lift motor to use encoder
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //declare subExtendMotor Stuff

        subExtendMotor = OpMode_.hardwareMap.dcMotor.get("subExtendMotor");

        subExtendMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        subExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        subExtendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        subExtendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       /* //declare intakeMotor Stuff

        intakeMotor = OpMode_.hardwareMap.dcMotor.get("intakeMotor");

        //intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/

        //declare hangMotor Stuff
        hangMotor = OpMode_.hardwareMap.dcMotor.get("hangMotor");

        hangMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //make hang motor use encoder stuff
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //declare claw servo
        clawServo = OpMode_.hardwareMap.servo.get("bucketServo");
        clawServo.setPosition(CLAW_SERVO_ENGAGED);

        //declare hang servo
        hangServo = OpMode_.hardwareMap.servo.get("hangServo");
        hangServo.setPosition(HANG_SERVO_LATCHED);

        //declare Sample Servo stuff
        sampleServo = OpMode_.hardwareMap.servo.get("sampleServo");
        sampleServo.setPosition(.5);


        //declare intakeServo

        intakeServo = OpMode_.hardwareMap.servo.get("intakeServo");
        intakeServo.setPosition(INTAKE_SERVO_INIT);

        //initialize IMU
        IMU.Parameters newParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.DOWN)
        );
        imu = OpMode_.hardwareMap.get(IMU.class,"imu");
        imu.initialize(newParameters);


        //Blinkin LED stuff
        blinkinLedDriver = OpMode_.hardwareMap.get(RevBlinkinLedDriver.class, "blinkin"); //Setting led map

        //winkinLedDriver = OpMode_.hardwareMap.get(RevBlinkinLedDriver.class, "winkin"); //2nd LED map


        //LED patterns
        autoPattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
        redPattern = RevBlinkinLedDriver.BlinkinPattern.BREATH_RED;
        bluePattern = RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE;
        endgamePattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD;

        //set the blinkin's color
        blinkinLedDriver.setPattern(bluePattern);

        //touch sensors
        liftTouch = OpMode_.hardwareMap.touchSensor.get("liftTouch");


        rightBase = OpMode_.hardwareMap.touchSensor.get("rightBase");
        leftBase = OpMode_.hardwareMap.touchSensor.get("leftBase");
        subTouch = OpMode_.hardwareMap.touchSensor.get("subTouch");


    }
}
