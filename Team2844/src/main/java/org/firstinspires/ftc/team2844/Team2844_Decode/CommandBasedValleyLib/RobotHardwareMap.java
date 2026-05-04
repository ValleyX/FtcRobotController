package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBasedValleyLib;

/*
 * ==============================================================================
 * PEDRO PATHING EXTRACTION DATA:
 * ==============================================================================
 * The following constants from your Constants.java are highly relevant for
 * tuning and configuring Pedro Pathing (e.g., inside FollowerConstants.java):
 * * ROBOT DIMENSIONS (For Bounding Box & Centricity):
 * - Width: 17.0 inches (Constants.BOT_WIDTH)
 * - Length: 17.75 inches (Constants.BOT_LENGTH)
 *
 * TUNING MULTIPLIERS:
 * - Lateral/Strafe Multiplier: 1.35 (Constants.STRAFE_CORRECTION)
 *
 * DRIVE MOTORS (For hardware configuration):
 * - leftFront  : Port CM0 ("leftFront")
 * - leftBack   : Port CM1 ("leftBack")
 * - rightBack  : Port CM2 ("rightBack")
 * - rightFront : Port CM3 ("rightFront")
 *
 * LOCALIZATION SENSOR (Pinpoint):
 * - Device Name: "pinpoint" (Constants.CBUS1)
 * ==============================================================================
 */

// Pedro Pathing Equivalent for Roadrunner's Pose2d
import com.pedropathing.geometry.Pose;

// ValleyLib Equivalents (replacing com.arcrobotics.ftclib)
import com.vcs.valleylib.ftc.hardware.*;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBasedValleyLib.MotorExPair;

public class RobotHardwareMap {

    /* ------------------- Motor Declarations -------------------*/
    public Motor frontLeft, frontRight, backLeft, backRight;
    public MotorGroup leftMotorGroup, rightMotorGroup;

    public MotorEx shooterLeft, shooterRight;
    public MotorExPair shooterPair;

    public Motor intakeMotor;
    public Motor tFeed;

    /* ------------------- Servo Declarations ------------------- */
    public Servo hoodAim;
    public Servo axon;
    public Servo kickerRotate;
    public Servo spindexer;

    public CRServo kickerSpin, sFeed;

    /* ------------------- Sensors & Vision ------------------- */
    public Limelight3A limelight;
    public GoBildaPinpointDriver pinpoint;

    public DigitalChannel topBreak;
    public DigitalChannel intakeBB;
    public AnalogInput axonIn;

    /**
     * Standard Constructor
     */
    public RobotHardwareMap(HardwareMap hardwareMap) {
        initHardware(hardwareMap);
    }

    /**
     * Auto Constructor (Allows passing in Pedro Pathing Pose)
     */
    public RobotHardwareMap(HardwareMap hardwareMap, Pose pose) {
        initHardware(hardwareMap);
        // Note: You can use the `pose` parameter here to initialize
        // the localizer's starting position if you decide to handle that natively.
    }

    /**
     * Centralized hardware initialization
     */
    private void initHardware(HardwareMap hardwareMap) {
        /* -------------- Motors -------------- */

        // ----- Shooter Motors ----- //
        shooterLeft = new MotorEx(hardwareMap, Constants.EM3);
        shooterRight = new MotorEx(hardwareMap, Constants.EM2);
        shooterLeft.setInverted(true);

        shooterPair = new MotorExPair(this.shooterLeft, this.shooterRight);
        shooterPair.setRunMode(Motor.RunMode.RawPower);
        shooterPair.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        // ----- Shooter Feed Motor ----- //
        tFeed = new Motor(hardwareMap, Constants.EM0);
        tFeed.setInverted(true);
        tFeed.setRunMode(Motor.RunMode.RawPower);

        // ----- Intake ----- //
        intakeMotor = new Motor(hardwareMap, Constants.EM1);
        intakeMotor.setInverted(true);

        /* -------------- Servos -------------- */

        // ----- Aim Servos ----- //
        axon = hardwareMap.get(Servo.class, Constants.CS2);
        hoodAim = hardwareMap.get(Servo.class, Constants.CS0);

        // ----- Kicker Servos ----- //
        kickerRotate = hardwareMap.get(Servo.class, Constants.CS1);
        sFeed = hardwareMap.get(CRServo.class, Constants.CS4);
        kickerSpin = hardwareMap.get(CRServo.class, Constants.CS3);

        // ----- Spindexer Servo ----- //
        spindexer = hardwareMap.get(Servo.class, Constants.CS5);

        /* -------------- Sensors -------------- */

        topBreak = hardwareMap.get(DigitalChannel.class, Constants.CDI0);
        intakeBB = hardwareMap.get(DigitalChannel.class, Constants.CDI2);
        axonIn = hardwareMap.get(AnalogInput.class, Constants.CAI2);

        // ----- Limelight ----- //
        limelight = hardwareMap.get(Limelight3A.class, Constants.LL);

        // ----- Pinpoint ----- //
        // Uncomment to natively initialize within the hardware map
        // pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.CBUS1);
    }
}