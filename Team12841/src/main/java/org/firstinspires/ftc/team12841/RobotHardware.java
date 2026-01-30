package org.firstinspires.ftc.team12841;

import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team12841.configs.PanelsConfig;
import org.firstinspires.ftc.team12841.pedroPathing.Constants;

public class RobotHardware {

    public final OpMode opMode;

    /* ===================== CONSTANTS ===================== */

    public static final double SHOOTER_TICKS_PER_REV = 28.0;

    private static final double LIMELIGHT_YAW_OFFSET_DEG = 180.0; // LL faces backwards

    /* ===================== MOTORS ===================== */

    public DcMotorEx lf, lb, rf, rb;
    public DcMotorEx shooter;
    public DcMotorEx intake;

    public DcMotorEx leftOdo, rightOdo, strafeOdo;

    /* ===================== SERVOS ===================== */

    public Servo leftFlick, rightFlick;
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

        PIDFCoefficients original = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        configureMotors();
        initIMU();
        initLimelight();
        initPedro();
        follower = Constants.createFollower(opMode.hardwareMap);


        opMode.telemetry.addData("Old PIDF", "%.04f, %.04f, %.04f, %.04f", original.p, original.i, original.d, original.f);
        opMode.telemetry.addLine("Robot ready");
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

        leftOdo   = motor("intakeMotor");
        rightOdo  = motor("rightOdo");
        strafeOdo = motor("strafeOdo");

        leftFlick = opMode.hardwareMap.get(Servo.class, "leftFlick");
        rightFlick = opMode.hardwareMap.get(Servo.class, "rightFlick");
        turntableServo = opMode.hardwareMap.get(Servo.class, "turntableServo");
    }

    private DcMotorEx motor(String name) {
        return opMode.hardwareMap.get(DcMotorEx.class, name);
    }

    private void configureMotors() {

        lf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotorEx m : new DcMotorEx[]{lf, lb, rf, rb}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setVelocityPIDFCoefficients(
                999,   // kP
                0.0,    // kI
                0.0,    // kD
                1 // kF
        );
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        for (DcMotorEx odo : new DcMotorEx[]{leftOdo, rightOdo, strafeOdo}) {
            odo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            odo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private void initIMU() {
        imu = opMode.hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));
    }

    private void initLimelight() {
        try {
            limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0);
            limelight.start();
        } catch (Exception e) {
            limelight = null;
        }
    }

    private void initPedro() {
        follower = Constants.createFollower(opMode.hardwareMap);

        // RE-ASSERT shooter mode AFTER Pedro
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    /* ===================== SHOOTER (RPM) ===================== */

    public void setShooterRPM(double rpm) {
        shooter.setVelocity((rpm * SHOOTER_TICKS_PER_REV) / 60.0);
    }

    public void stopShooter() {
        shooter.setVelocity(0);
    }


    /* ===================== DRIVE ===================== */

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

    /* ===================== LIMELIGHT (BACKWARDS) ===================== */

    private void updateLL() {
        if (limelight != null) {
            llResult = limelight.getLatestResult();
        }
    }

    /** tx must be inverted because camera faces backwards */
    public double getTx() {
        updateLL();
        return (llResult != null && llResult.isValid())
                ? -llResult.getTx()
                : -999;
    }

    public double getDistance() {
        updateLL();
        return (llResult != null && llResult.isValid())
                ? 68.86747 * Math.pow(llResult.getTa(), -0.5169279)
                : -999;
    }

    /** Robot heading sent to LL must be offset by 180° */
    public void updateLLHeading() {
        if (limelight != null) {
            double correctedHeading =
                    headingDeg() + LIMELIGHT_YAW_OFFSET_DEG;
            limelight.updateRobotOrientation(correctedHeading);
        }
    }

    /* ===================== TURNING ===================== */

    public void turnToFree(double targetDeg, double speed) {
        double error = targetDeg - headingDeg();
        double power = clamp(error * PanelsConfig.LLPGAIN) * speed;
        addAlign(-power, -power, power, power);
    }

    public void alignWithLimelight(double speed) {
        double tx = getTx();

        if (tx == -999) {
            addAlign(0, 0, 0, 0);
            return;
        }

        double power = -(clamp(tx * PanelsConfig.LLPGAIN) * speed)                                     ;

        addAlign(-power, -power, power, power);
    }

    public double calculateRegression(double distance) {
        if(distance == -999)
            return 3500;
        double A = PanelsConfig.REGRESSION_A;
        double B = PanelsConfig.REGRESSION_B;
        double C = PanelsConfig.REGRESSION_C;
        return (A * (distance * distance)) + (B * distance) + C;
    }


    /* ===================== PEDRO ===================== */

    public Follower getFollower() {
        return follower;
    }

    public void resetHeading() {
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
