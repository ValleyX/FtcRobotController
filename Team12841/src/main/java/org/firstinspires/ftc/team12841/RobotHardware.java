package org.firstinspires.ftc.team12841;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.team12841.pedroPathing.Constants;
import com.pedropathing.follower.Follower;

public class RobotHardware {
    public final OpMode opMode_;
    private final ElapsedTime runtime = new ElapsedTime();

    // --- Drive Motors ---
    public DcMotorEx rfMotor;
    public DcMotorEx rbMotor;
    public DcMotorEx lfMotor;
    public DcMotorEx lbMotor;

    // --- Shooter Motor ---
    public DcMotorEx shooterMotor;

    // --- Servos ---
    public Servo turntableServo;
    public Servo shooterServo;

    // --- Sensors ---
    public IMU imu;
    public Limelight3A limelight3A;

    // --- Pedro Pathing ---
    private Follower follower;

    public RobotHardware(OpMode opMode) {
        this.opMode_ = opMode;

        // === Initialize Motors ===
        try {
            rfMotor = opMode_.hardwareMap.get(DcMotorEx.class, "rfMotor");
            rbMotor = opMode_.hardwareMap.get(DcMotorEx.class, "rbMotor");
            lfMotor = opMode_.hardwareMap.get(DcMotorEx.class, "lfMotor");
            lbMotor = opMode_.hardwareMap.get(DcMotorEx.class, "lbMotor");

            shooterMotor = opMode_.hardwareMap.get(DcMotorEx.class, "shooterMotor");
        } catch (Exception e) {
            opMode_.telemetry.addLine("Motor mapping error: check configuration names!");
        }

        // === Initialize Servos ===
        try {
            turntableServo = opMode_.hardwareMap.get(Servo.class, "turntableServo");
            shooterServo = opMode_.hardwareMap.get(Servo.class, "shooterServo");
        } catch (Exception e) {
            opMode_.telemetry.addLine("Servo mapping error: check servo names!");
        }

        // === Initialize IMU ===
        try {
            imu = opMode_.hardwareMap.get(IMU.class, "imu");
            RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP
            );
            imu.initialize(new IMU.Parameters(orientation));
        } catch (Exception e) {
            opMode_.telemetry.addLine("IMU not found or not initialized.");
        }

        // === Initialize Limelight 3A ===
      //  try {
      //      limelight3A = opMode_.hardwareMap.get(Limelight3A.class, "limelight");
      //      limelight3A.pipelineSwitch(0);  // default pipeline
      //      limelight3A.start();             // begin capturing
      //  } catch (Exception e) {
      //      opMode_.telemetry.addLine("Limelight3A not found. Skipping vision initialization.");
      //  }

        // === Motor Configuration ===
        try {
            lbMotor.setDirection(DcMotor.Direction.REVERSE);
            lfMotor.setDirection(DcMotor.Direction.REVERSE);
            rbMotor.setDirection(DcMotor.Direction.FORWARD);
            rfMotor.setDirection(DcMotor.Direction.FORWARD);

            for (DcMotorEx motor : new DcMotorEx[]{rfMotor, rbMotor, lbMotor}) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            if (lfMotor != null) {
                lfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (shooterMotor != null) {
                shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
        } catch (Exception e) {
            opMode_.telemetry.addLine("Motor direction/config setup failed.");
        }

        // === Pedro Pathing Follower ===
        try {
            follower = Constants.createFollower(opMode_.hardwareMap);
            opMode_.telemetry.addLine("Pedro Pathing initialized successfully");
        } catch (Exception e) {
            follower = null;
            opMode_.telemetry.addLine("Pedro Pathing not available — running basic mode");
        }

        opMode_.telemetry.update();
    }

    /** Safe getter for Pedro follower */
    public Follower getFollower() {
        return follower;
    }

    /** Reset IMU heading for field centric */
    public void resetIMU() {
        if (imu != null) imu.resetYaw();
    }
}
