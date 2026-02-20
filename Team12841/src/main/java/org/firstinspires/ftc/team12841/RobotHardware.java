package org.firstinspires.ftc.team12841;

// Pedro Pathing
import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team12841.pedroPathing.Constants;

// Limelight
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

// IMU
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

// Motors and OpModes
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

// My Files
import org.firstinspires.ftc.team12841.configs.PanelsConfig;

public class RobotHardware {

    public final OpMode opMode;

    /* ===================== CONSTANTS ===================== */

    public static final double SHOOTER_TICKS_PER_REV = 28.0;
    public static double rpm = 2400;

    private static final long TAG_MEMORY_TIMEOUT_MS = 5000;

    /* ===================== MOTORS ===================== */

    public DcMotorEx lf, lb, rf, rb;
    public DcMotorEx shooter;
    public DcMotorEx intake;
    public DcMotorEx flick;

    public DcMotorEx leftOdo, rightOdo, strafeOdo;

    /* ===================== SENSORS ===================== */

    public IMU imu;
    public Limelight3A limelight;
    private LLResult llResult;
    private DigitalChannel beambreak;

    /* ===================== PEDRO ===================== */

    private Follower follower;

    /* ===================== DRIVE STATE ===================== */

    private double lfDrive, lbDrive, rfDrive, rbDrive;
    private double lfAlign, lbAlign, rfAlign, rbAlign;

    /* ===================== LIMELIGHT MEMORY ===================== */

    public Double lastSeenTagFieldX = null;
    public Double lastSeenTagFieldY = null;
    private long lastSeenTagTimeMs = 0;

    /* ===================== CONSTRUCTOR ===================== */

    public RobotHardware(OpMode opMode) {
        this.opMode = opMode;

        mapHardware();
        configureMotors();
        initIMU();
        initLimelight();
        initPedro();

        limelight.pipelineSwitch(0);
    }

    /* ===================== INITIALIZATION ===================== */

    private void mapHardware() {
        lf = motor("lfMotor");
        lb = motor("lbMotor");
        rf = motor("rfMotor");
        rb = motor("rbMotor");

        shooter = motor("shooterMotor");
        intake  = motor("intakeMotor");
        flick   = motor("flickMotor");

        leftOdo   = motor("lfMotor");
        rightOdo  = motor("rbMotor");
        strafeOdo = motor("lbMotor");

        beambreak = opMode.hardwareMap.get(DigitalChannel.class, "distanceSensor");
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
        shooter.setVelocityPIDFCoefficients(375, 0, 0, 0.8);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flick.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flick.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    /* ===================== LIMELIGHT ===================== */

    private void updateLL() {
        if (limelight != null) {
            llResult = limelight.getLatestResult();
        }
    }

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

    /* ===================== TAG MEMORY ===================== */

    public void updateAndCacheAprilTag() {
        updateLL();

        if (llResult == null || !llResult.isValid()) return;

        Pose pose = follower.getPose();

        double distance = getDistance();
        if (distance == -999) return;

        double txRad = Math.toRadians(-llResult.getTx());
        double headingRad = Math.toRadians(pose.getHeading());
        double globalAngle = headingRad + txRad;

        lastSeenTagFieldX = pose.getX() + distance * Math.sin(globalAngle);
        lastSeenTagFieldY = pose.getY() + distance * Math.cos(globalAngle);
    }

    private void rotateToCachedTag(double speed) {
        if (lastSeenTagFieldX == null || lastSeenTagFieldY == null) {
            // no target stored
            return;
        }

        Pose pose = follower.getPose();

        double dx = lastSeenTagFieldX - pose.getX();
        double dy = lastSeenTagFieldY - pose.getY();

        // compute desired heading to tag (radians)
        double targetHeading = Math.atan2(dy, dx);

        // compute heading error normalized
        double currentHeading = pose.getHeading();
        double error = Math.atan2(Math.sin(targetHeading - currentHeading),
                Math.cos(targetHeading - currentHeading));

        // proportional turn command
        double rotPower = clamp(error * PanelsConfig.LLPGAIN) * speed;

        // tell Pedro to apply heading control
        follower.setTeleOpDrive(0, 0, -clamp(rotPower * 50), false);
    }


    /* ===================== TURNING ===================== */

    public void alignWithLimelight(double speed) {
        updateAndCacheAprilTag();

        double tx = getTx();

        if (tx != -999) {
            double power = -(clamp(tx * PanelsConfig.LLPGAIN) * speed);
            addAlign(-power, -power, power, power);
        } else {
            rotateToCachedTag(speed);
        }
        return;
    }

    /* ===================== SHOOTER ===================== */

    public void setShooterRPM(double rpm) {
        shooter.setVelocity((rpm * SHOOTER_TICKS_PER_REV) / 60.0);
    }

    public void stopShooter() {
        shooter.setVelocity(0);
    }

    public double calculateRegression() {
        double distance = getDistance();
        if (distance == -999) return rpm;

        double A = PanelsConfig.REGRESSION_A;
        double B = PanelsConfig.REGRESSION_B;
        double C = PanelsConfig.REGRESSION_C;
        double D = PanelsConfig.REGRESSION_D;
        double F = PanelsConfig.REGRESSION_F;

        rpm = (A * (Math.pow(distance, 4))) + (B * (Math.pow(distance, 3))) + (C * (Math.pow(distance, 2))) + (D * distance) + F;
        return rpm;
    }

    public boolean isBroken() {
        return beambreak.getState();
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
}
