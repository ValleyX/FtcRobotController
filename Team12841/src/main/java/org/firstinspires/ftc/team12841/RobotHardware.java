package org.firstinspires.ftc.team12841;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team12841.pedroPathing.Constants; // use your constants class

import com.pedropathing.follower.Follower;

public class RobotHardware {
    public final OpMode opMode_;
    private final ElapsedTime runtime = new ElapsedTime();

    // --- Drive Motors ---
    public DcMotorEx rfMotor, rbMotor, lfMotor, lbMotor;
    public DcMotorEx shooterMotor;

    // --- Servos ---
    public Servo turntableServo, shooterServo;

    // --- Sensors ---
    public IMU imu;
    public Limelight3A limelight3A;

    // --- Pedro Pathing ---
    public Follower follower;

    public RobotHardware(OpMode opMode) {
        this.opMode_ = opMode;

        // --- Initialize Core Hardware ---
        rfMotor = opMode_.hardwareMap.get(DcMotorEx.class, "rfMotor");
        rbMotor = opMode_.hardwareMap.get(DcMotorEx.class, "rbMotor");
        lfMotor = opMode_.hardwareMap.get(DcMotorEx.class, "lfMotor");
        lbMotor = opMode_.hardwareMap.get(DcMotorEx.class, "lbMotor");

        shooterMotor = opMode_.hardwareMap.get(DcMotorEx.class, "shooterMotor");

        turntableServo = opMode_.hardwareMap.get(Servo.class, "turntableServo");
        shooterServo = opMode_.hardwareMap.get(Servo.class, "shooterServo");

        imu = opMode_.hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(orientation));

        // --- Motor Configuration ---
        lbMotor.setDirection(DcMotor.Direction.REVERSE);
        lfMotor.setDirection(DcMotor.Direction.REVERSE);
        rbMotor.setDirection(DcMotor.Direction.FORWARD);
        rfMotor.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotorEx m : new DcMotorEx[]{rfMotor, rbMotor, lfMotor, lbMotor}) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // --- Initialize Pedro Follower ---
        try {
            follower = Constants.createFollower(opMode_.hardwareMap);
            opMode_.telemetry.addLine("Pedro Pathing initialized successfully");
        } catch (Exception e) {
            follower = null;
            opMode_.telemetry.addLine("Pedro Pathing not available — running basic mode");
        }

        opMode_.telemetry.update();
    }

    /** Returns the Pedro follower if available */
    public Follower getFollower() {
        return follower;
    }

    /** Reset IMU heading for normal OpModes */
    public void resetIMU() {
        imu.resetYaw();
    }
}
