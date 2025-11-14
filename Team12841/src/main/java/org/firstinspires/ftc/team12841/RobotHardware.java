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

    public final OpMode opMode;
    private final ElapsedTime runtime = new ElapsedTime();

    // --- Drive Motors ---
    public DcMotorEx rfMotor, rbMotor, lfMotor, lbMotor;

    // Shooter
    public DcMotorEx shooterMotor;

    // --- Odometry Encoders (Expansion Hub 1–3) ---
    public DcMotorEx leftOdo;
    public DcMotorEx rightOdo;
    public DcMotorEx strafeOdo;

    // Servos
    public Servo turntableServo, shooterServo;

    // Sensors
    public IMU imu;
    public Limelight3A limelight;

    // Pedro
    private Follower follower;

    public RobotHardware(OpMode opMode) {
        this.opMode = opMode;

        try {
            // Drive motors
            rfMotor = opMode.hardwareMap.get(DcMotorEx.class, "rfMotor");
            rbMotor = opMode.hardwareMap.get(DcMotorEx.class, "rbMotor");
            lfMotor = opMode.hardwareMap.get(DcMotorEx.class, "lfMotor");
            lbMotor = opMode.hardwareMap.get(DcMotorEx.class, "lbMotor");

            // Shooter
            shooterMotor = opMode.hardwareMap.get(DcMotorEx.class, "shooterMotor");

            // ✔ Correct odometry names
            leftOdo = opMode.hardwareMap.get(DcMotorEx.class, "leftOdo");
            rightOdo = opMode.hardwareMap.get(DcMotorEx.class, "rightOdo");
            strafeOdo = opMode.hardwareMap.get(DcMotorEx.class, "strafeOdo");

        } catch (Exception e) {
            opMode.telemetry.addLine("ERROR: Motor/Odo mapping failed!");
        }

        // Servos
        try {
            shooterServo = opMode.hardwareMap.get(Servo.class, "shooterServo");
            turntableServo = opMode.hardwareMap.get(Servo.class, "turntableServo");
        } catch (Exception e) {
            opMode.telemetry.addLine("ERROR: Servo mapping failed!");
        }

        // IMU
        try {
            imu = opMode.hardwareMap.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )
            ));
        } catch (Exception e) {
            opMode.telemetry.addLine("ERROR: IMU failed!");
        }

        // Drive config
        try {
            lfMotor.setDirection(DcMotor.Direction.REVERSE);
            lbMotor.setDirection(DcMotor.Direction.REVERSE);

            rfMotor.setDirection(DcMotor.Direction.FORWARD);
            rbMotor.setDirection(DcMotor.Direction.FORWARD);

            for (DcMotorEx m : new DcMotorEx[]{lfMotor, lbMotor, rfMotor, rbMotor}) {
                m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Encoders should NOT output power
            DcMotorEx[] odos = {leftOdo, rightOdo, strafeOdo};
            for (DcMotorEx odo : odos) {
                odo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                odo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                odo.setPower(0);
            }

        } catch (Exception e) {
            opMode.telemetry.addLine("ERROR: motor/encoder config failed!");
        }

        // Pedro init
        try {
            follower = Constants.createFollower(opMode.hardwareMap);
            opMode.telemetry.addLine("Pedro Pathing OK");
        } catch (Exception e) {
            follower = null;
            opMode.telemetry.addLine("Pedro FAILED");
        }

        opMode.telemetry.update();
    }

    public Follower getFollower() {
        return follower;
    }

    public void resetIMU() {
        if (imu != null) imu.resetYaw();
    }
}
