package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.MotorExGroup;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.DriveSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.SensorSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterFeedSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SpindexerSubsystem;

import java.util.ArrayList;
import java.util.Arrays;

public class Subsystems {

    /* ------------------- Motor Declarations -------------------*/
    /**
     * Drive motor parameter pass-ins
     */
    private Motor frontLeft, frontRight, backLeft, backRight;
    /**
     * Drive motor group parameter pass-ins
     */
    private MotorGroup leftMotorGroup, rightMotorGroup;

    /**
     * Motors for shooter subsystem pass-ins (one the motors in the pair of shooter motors)
     */
    private MotorEx shooterLeft, shooterRight;
    /**
     * Motor group of the shooters, the left shooter should be inverted
     */
    private MotorExGroup shooterPair;

    /**
     * Intake Motor
     */
    private Motor intakeMotor;

    /* ------------------- Servo Declarations ------------------- */
    /**
     * Aiming servo object parameter pass-ins
     */
    private Servo hoodAim;
    private CRServo turretAim;

    /**
     * Servo objects for passing into kicksubsystem
     */
    private Servo kickerRotate;
    /**
     * CRServo objects for passing into kicksubsystem
     */
    private CRServo kickerSpin, sFeed;

    /**
     * Servo objects for passing into spindexerSubsystem
     */
    private Servo spindexer;

    /***/
    private Motor tFeed;

    /* ------------------- Limelight ------------------- */
    private Limelight3A limelight;

    /* ------------------- Subsystem Declarations ------------------- */
    /**
     * Drive Subsystem
     */
    //private TankDriveSubsystem tankDriveSubsystem;
    public DriveSubsystem mecDriveSubsystem;
    /**
     * Intake Subsystem
     */
    public IntakeSubsystem intakeSubsystem;
    /**
     * Aim Subsystem
     */
    public AimSubsystem aimSubsystem;
    /**
     * Shooter Subsystem
     */
    public ShooterSubsystem shooterSubsystem;
    /**
     * Shooter feed Subsystem
     */
    public ShooterFeedSubsystem shooterFeedSubsystem;
    /**
     * Kick Subsystem
     */
    public KickSubsystem kickSubsystem;
    /**
     * Spindexer Subsystem
     */
    public SpindexerSubsystem spindexerSubsystem;
    /**Limelight Subsystem*/
    //private LimelightSubsystem limelightSubsystem;
    /**
     * Pinpoint Subsystem
     */
    public SensorSubsystem sensorSubsystem;


    /* ------------------- Sensor Declarations ------------------- */
    //private IMU turretIMU;
    //private Motor turretServoEncoder;
    private GoBildaPinpointDriver pinpoint;
    private DigitalChannel topBreak;
    private DigitalChannel intakeBB;
    private AnalogInput axonIn;

    public Subsystems(HardwareMap hardwareMap, int pipelineNum) {
        /* -------------- Hardware Maps -------------- */

        // ----- Drive Motors ----- //
        //Map HW motors to variables
        frontLeft = new Motor(hardwareMap, Constants.CM0);
        frontRight = new Motor(hardwareMap, Constants.CM3);
        backLeft = new Motor(hardwareMap, Constants.CM1);
        backRight = new Motor(hardwareMap, Constants.CM2);

        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        backLeft.setInverted(false);
        backRight.setInverted(false);

        //Create motor group with Front as the leader
        //leftMotorGroup = new MotorGroup(this.frontLeft,this.backLeft);
        //rightMotorGroup = new MotorGroup(this.frontRight,this.backRight);

//        rightMotorGroup.setInverted(true);

        // ----- Shooter Motors ----- //
        shooterLeft = new MotorEx(hardwareMap, Constants.EM3);
        shooterRight = new MotorEx(hardwareMap, Constants.EM2);
        shooterLeft.setInverted(true);

        shooterPair = new MotorExGroup(new ArrayList<>(Arrays.asList(this.shooterLeft, this.shooterRight)));
        shooterPair.setRunMode(Motor.RunMode.RawPower);
        shooterPair.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        // ----- Shooter Feed Motor ----- //
        tFeed = new Motor(hardwareMap, Constants.EM0);
        tFeed.setInverted(true);
        tFeed.setRunMode(Motor.RunMode.RawPower);

        // ----- Aim Servos ----- //
        turretAim = hardwareMap.get(CRServo.class, Constants.CS2);
        hoodAim = hardwareMap.get(Servo.class, Constants.CS0);

        // ----- Intake ----- //
        intakeMotor = new Motor(hardwareMap, Constants.EM1);
        intakeMotor.setInverted(true);

        intakeBB = hardwareMap.get(DigitalChannel.class, Constants.CDI2);

        // ----- Kicker Servos ----- //
        kickerRotate = hardwareMap.get(Servo.class, Constants.CS1);
        kickerRotate.setPosition(0.0);
        sFeed = hardwareMap.get(CRServo.class, Constants.CS4);
        kickerSpin = hardwareMap.get(CRServo.class, Constants.CS3);


        // ----- Spindexer Servo ----- //
        spindexer = hardwareMap.get(Servo.class, Constants.CS5);

        // ----- Limelight ----- //
        limelight = hardwareMap.get(Limelight3A.class, Constants.LL);


        /* ---------- Pinpoint ----------*/
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.CBUS0);

        /* -------------- Sensors -------------- */
        topBreak = hardwareMap.get(DigitalChannel.class, Constants.CDI0);
        axonIn = hardwareMap.get(AnalogInput.class, Constants.CAI2);

        /* -------------- Subsystems -------------- */
        /* 1. drive
         * 2. intake
         * 3. Kick
         * 4. Spindexer
         * 5. Shooter
         * 6. ShooterFeed
         * 7. Aim
         * 8. Sensor */

        //1. drive subsystem
        //tankDriveSubsystem = new TankDriveSubsystem(leftMotorGroup,rightMotorGroup);
        mecDriveSubsystem = new DriveSubsystem(backLeft, frontRight, frontLeft, backRight);
        //2. Intake subsystem
        intakeSubsystem = new IntakeSubsystem(intakeMotor, intakeBB);
        //3. Kick subsystem
        kickSubsystem = new KickSubsystem(kickerRotate, kickerSpin, sFeed);
        //4. Spindexer subsystem
        spindexerSubsystem = new SpindexerSubsystem(hardwareMap, spindexer);
        //5. Shooter subsystem
        shooterSubsystem = new ShooterSubsystem(shooterPair);
        //6. ShooterFeed subsystem
        shooterFeedSubsystem = new ShooterFeedSubsystem(tFeed, topBreak);
        //7. Aim subsystem
        aimSubsystem = new AimSubsystem(hoodAim, turretAim, axonIn);
        //8. Sensor Subsystem
        sensorSubsystem = new SensorSubsystem(pinpoint, limelight, pipelineNum);
        sensorSubsystem.setPinpointPose(new Pose2D(DistanceUnit.INCH, SavedVars.startingX, SavedVars.startingY, AngleUnit.DEGREES, SavedVars.startingHeading - 90.0));
    }



//AUTO CONSTRUCTOR (DOESN'T USE MECDRIVE SO ROADRUNNER CAN USE MECANUM EXAMPLE)

    public Subsystems(HardwareMap hardwareMap, int pipelineNum, boolean auto) {
        /* -------------- Hardware Maps -------------- */

        // ----- Shooter Motors ----- //
        shooterLeft = new MotorEx(hardwareMap, Constants.EM3);
        shooterRight = new MotorEx(hardwareMap, Constants.EM2);
        shooterLeft.setInverted(true);

        shooterPair = new MotorExGroup(new ArrayList<>(Arrays.asList(this.shooterLeft, this.shooterRight)));
        shooterPair.setRunMode(Motor.RunMode.RawPower);
        shooterPair.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        // ----- Shooter Feed Motor ----- //
        tFeed = new Motor(hardwareMap, Constants.EM0);
        tFeed.setInverted(true);
        tFeed.setRunMode(Motor.RunMode.RawPower);

        // ----- Aim Servos ----- //
        turretAim = hardwareMap.get(CRServo.class, Constants.CS2);
        hoodAim = hardwareMap.get(Servo.class, Constants.CS0);

        // ----- Intake ----- //
        intakeMotor = new Motor(hardwareMap, Constants.EM1);
        intakeMotor.setInverted(true);

        intakeBB = hardwareMap.get(DigitalChannel.class, Constants.CDI2);

        // ----- Kicker Servos ----- //
        kickerRotate = hardwareMap.get(Servo.class, Constants.CS1);
        kickerRotate.setPosition(0.0);
        sFeed = hardwareMap.get(CRServo.class, Constants.CS4);
        kickerSpin = hardwareMap.get(CRServo.class, Constants.CS3);


        // ----- Spindexer Servo ----- //
        spindexer = hardwareMap.get(Servo.class, Constants.CS5);

        /* -------------- Sensors -------------- */
        topBreak = hardwareMap.get(DigitalChannel.class, Constants.CDI0);
        axonIn = hardwareMap.get(AnalogInput.class, Constants.CAI2);

        /* -------------- Subsystems -------------- */
        /* 1. intake
         * 2. Kick
         * 3. Spindexer
         * 4. Shooter
         * 5. ShooterFeed
         * 6. Aim */

        //1. Intake subsystem
        intakeSubsystem = new IntakeSubsystem(intakeMotor, intakeBB);
        //2. Kick subsystem
        kickSubsystem = new KickSubsystem(kickerRotate, kickerSpin, sFeed);
        //3. Spindexer subsystem
        spindexerSubsystem = new SpindexerSubsystem(hardwareMap, spindexer);
        //4. Shooter subsystem
        shooterSubsystem = new ShooterSubsystem(shooterPair);
        //5. ShooterFeed subsystem
        shooterFeedSubsystem = new ShooterFeedSubsystem(tFeed, topBreak);
        //6. Aim subsystem
        aimSubsystem = new AimSubsystem(hoodAim, turretAim, axonIn);
        //7. Sensor Subsystem
        sensorSubsystem = new SensorSubsystem(limelight, pipelineNum);
    }
}
