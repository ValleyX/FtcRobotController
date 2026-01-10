package org.firstinspires.ftc.team12841;

import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team12841.configs.TeleOpConfig;
import org.firstinspires.ftc.team12841.pedroPathing.Constants;

public class RobotHardware {

    public final OpMode opMode;

    /* ===================== CONSTANTS ===================== */

    private static final double SHOOTER_TICKS_PER_REV = 5880.0;

    /* ===================== MOTORS ===================== */

    // Drive
    public DcMotorEx lf, lb, rf, rb;

    // Mechanisms
    public DcMotorEx shooter;
    public DcMotorEx intake;

    // Odometry
    public DcMotorEx leftOdo, rightOdo, strafeOdo;

    /* ===================== SERVOS ===================== */

    public Servo shooterServo;
    public Servo turntableServo;

    /* ===================== SENSORS ===================== */

    public IMU imu;
    public Limelight3A limelight;
    private LLResult llResult;

    /* ===================== PEDRO ===================== */

    private Follower follower;

    /* ===================== DRIVE STATE ===================== */

    private double lfDrive, lbDrive, rfDrive, rbDrive;
    private double lfAlign, lbAlign, rfAlign, rbAlign;

    /* ===================== CONSTRUCTOR ===================== */

    public RobotHardware(OpMode opMode) {
        this.opMode = opMode;

        mapHardware();
        configureMotors();
        initIMU();
        initLimelight();
        initPedro();

        opMode.telemetry.addLine("All systems GO, Houston we are ready for launch!");
        opMode.telemetry.update();
    }

    /* ===================== INITIALIZATION ===================== */

    private void mapHardware() {
        lf = motor("lfMotor");
        lb = motor("lbMotor");
        rf = motor("rfMotor");
        rb = motor("rbMotor");

        shooter = motor("shooterMotor");
        intake  = motor("intakeMotor");

        leftOdo   = motor("leftOdo");
        rightOdo  = motor("rightOdo");
        strafeOdo = motor("strafeOdo");

        shooterServo   = opMode.hardwareMap.get(Servo.class, "shooterServo");
        turntableServo = opMode.hardwareMap.get(Servo.class, "turntableServo");
    }

    private DcMotorEx motor(String name) {
        return opMode.hardwareMap.get(DcMotorEx.class, name);
    }

    private void configureMotors() {

        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotorEx m : new DcMotorEx[]{lf, lb, rf, rb}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        for (DcMotorEx odo : new DcMotorEx[]{leftOdo, rightOdo, strafeOdo}) {
            odo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            odo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        opMode.telemetry.addLine("Motors Configured");
    }

    private void initIMU() {
        imu = opMode.hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));
        opMode.telemetry.addLine("IMU Initialized");
    }

    private void initLimelight() {
        try {
            limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0);
            limelight.start();
        } catch (Exception e) {
            limelight = null;
        }
        opMode.telemetry.addLine("LL Initialized");
    }

    private void initPedro() {
        follower = Constants.createFollower(opMode.hardwareMap);
        opMode.telemetry.addLine("Pedro Pathing Initialized");
    }

    /* ===================== SHOOTER (RPM CONTROL) ===================== */

    public void setShooterRPM(double rpm) {
        shooter.setVelocity(rpmToTicksPerSec(rpm));
    }

    public void stopShooter() {
        shooter.setVelocity(0);
    }

    private double rpmToTicksPerSec(double rpm) {
        return (rpm * SHOOTER_TICKS_PER_REV) / 60.0;
    }

    /* ===================== INTAKE ===================== */

    public void runIntake(double power) {
        intake.setPower(clamp(power));
    }

    public void stopIntake() {
        intake.setPower(0);
    }

    /* ===================== DRIVE ===================== */

    public void addDrive(double lf, double lb, double rf, double rb) {
        lfDrive = lf;
        lbDrive = lb;
        rfDrive = rf;
        rbDrive = rb;
        applyDrive();
    }

    public void addAlign(double lf, double lb, double rf, double rb) {
        lfAlign = lf;
        lbAlign = lb;
        rfAlign = rf;
        rbAlign = rb;
        applyDrive();
    }

    private void applyDrive() {
        lf.setPower(clamp(lfDrive + lfAlign));
        lb.setPower(clamp(lbDrive + lbAlign));
        rf.setPower(clamp(rfDrive + rfAlign));
        rb.setPower(clamp(rbDrive + rbAlign));
    }

    private double clamp(double v) {
        return Math.max(-1.0, Math.min(1.0, v));
    }

    /* ===================== IMU ===================== */

    public double headingDeg() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public double headingRad() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void resetIMU() {
        imu.resetYaw();
    }

    /* ===================== LIMELIGHT ===================== */

    private void updateLL() {
        if (limelight != null) {
            llResult = limelight.getLatestResult();
        }
    }

    public double getTx() {
        updateLL();
        return (llResult != null && llResult.isValid()) ? llResult.getTx() : -999;
    }

    public double getDistance() {
        updateLL();
        return (llResult != null && llResult.isValid())
                ? 68.86747 * Math.pow(llResult.getTa(), -0.5169279)
                : -999;
    }

    public void updateLLHeading() {
        if (limelight != null) {
            limelight.updateRobotOrientation(headingDeg());
        }
    }

    /* ===================== TURNING ===================== */

    public void turnToFree(double targetDeg, double speed) {
        double error = targetDeg - headingDeg();
        double power = clamp(error * TeleOpConfig.LLPGAIN) * speed;
        addAlign(-power, -power, power, power);
    }

    /* ===================== PEDRO ===================== */

    public Follower getFollower() {
        return follower;
    }

    public void resetPedroHeading() {
        imu.resetYaw();
        Pose pose = follower.getPose();
        follower.setPose(new Pose(pose.getX(), pose.getY(), 0));
    }

    /* ===================== FULL STOP ===================== */

    public void fullBrake() {
        for (DcMotorEx m : new DcMotorEx[]{lf, lb, rf, rb}) {
            m.setPower(0);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
}
