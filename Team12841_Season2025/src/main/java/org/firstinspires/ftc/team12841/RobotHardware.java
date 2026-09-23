package org.firstinspires.ftc.team12841;

import static org.firstinspires.ftc.team12841.configs.PanelsConfig.IN_POS;
import static org.firstinspires.ftc.team12841.configs.PanelsConfig.OUT_POS;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.team12841.configs.PanelsConfig; // Retained from new file for LLPGAIN
import org.firstinspires.ftc.team12841.pedroPathing.Constants; // Retained from new file for Follower setup

import java.util.List;

public class RobotHardware {

    /*
     * Core & Globals
     */
    private final LinearOpMode opMode_;
    private final ElapsedTime runtime = new ElapsedTime();
    public Follower follower;

    /*
     * Drivetrain Motors & Sensors
     */
    public DcMotorEx rightBackDrive;
    public DcMotorEx leftBackDrive;
    public DcMotorEx rightFrontDrive;
    public DcMotorEx leftFrontDrive;

    public DcMotorEx leftOdo;
    public DcMotorEx rightOdo;
    public DcMotorEx strafeOdo;

    public IMU imu;
    public VoltageSensor batteryVoltSensor;

    // Motor speed variables
    public double lFDriveSpeed = 0.0;
    public double rFDriveSpeed = 0.0;
    public double lBDriveSpeed = 0.0;
    public double rBDriveSpeed = 0.0;

    public double lFAlignSpeed = 0.0;
    public double rFAlignSpeed = 0.0;
    public double lBAlignSpeed = 0.0;
    public double rBAlignSpeed = 0.0;

    public final double TURN_THRESH = 1.8;
    public final double SLOW_THRESH = 15;
    public final double PGAIN = 0.04;

    /*
     * Shooter, Intake & Flick Motors
     */
    public DcMotorEx shooterMotorBilda;
    public DcMotorEx shooterMotorRev;
    public DcMotorEx intakeMotor;
    //public DcMotorEx flickMotor;

    /*
     * Servos
     */
    public Servo timerLight;
    public Servo fullLight;
    private Servo blockSer;
    private Servo hoodSer;
    private Servo ballStop;

    // Servo Constants
    private boolean servoClosed;

    /*
     * BeamBreaks / DigitalChannels
     */
    private DigitalChannel BB0;
    private DigitalChannel BB1;
    private DigitalChannel BB2;
    private DigitalChannel BB3;
    private DigitalChannel gobuildaBB;
    private DigitalChannel gobuildaBB1;
    private DigitalChannel beamBreak; // Added from new file

    /*
     * Shooter Conversions & Tracking
     */

    public PIDFCoefficients shooterCoefficientsBilda;
    public PIDFCoefficients shooterCoefficientsRev;
    public PIDFCoefficients newShooterCoefficients;

    /*
     * Limelight
     */
    public Limelight3A limelight;
    private LLResult llResult;
    private boolean pipelineCalled = false;

    private static double velThresh = PanelsConfig.velThresh;
    public static double ENCODER_TICS = PanelsConfig.ENCODER_TICS;
    private static double hoodAim = PanelsConfig.hoodAim;
    public static double VEL_BOTTOM_THRESH = PanelsConfig.VEL_BOTTOM_THRESH;
    public static double ghettoIn = PanelsConfig.ghettoIn;
    public static double ghettoOut = PanelsConfig.ghettoOut;

    /**
     * Primary Constructor
     */
    public RobotHardware(LinearOpMode opMode) {
        opMode_ = opMode;

        /* --- DRIVETRAIN INIT --- */
        rightFrontDrive = opMode_.hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBackDrive = opMode_.hardwareMap.get(DcMotorEx.class, "rightBack");
        leftFrontDrive = opMode_.hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBackDrive = opMode_.hardwareMap.get(DcMotorEx.class, "leftBack");

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotorEx m : new DcMotorEx[]{leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Map Odometry
        leftOdo = leftFrontDrive;
        rightOdo = rightBackDrive;
        strafeOdo = leftBackDrive;
        resetOdometry();

        batteryVoltSensor = opMode_.hardwareMap.voltageSensor.iterator().next();

        // IMU Init (Updated to RIGHT / UP based on new config)
        imu = opMode_.hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );

        /* --- PEDROPATHING INIT --- */
        follower = Constants.createFollower(opMode_.hardwareMap);


        /* --- LIGHT SERVOS --- */
        timerLight = opMode_.hardwareMap.get(Servo.class, "timerLight"); // Also maps to "light" conceptually
        timerLight.setPosition(0.611);

        fullLight = opMode_.hardwareMap.get(Servo.class, "fullLight");
        fullLight.setPosition(0.611);


        /* --- SHOOTER, INTAKE, FLICK INIT --- */
        shooterMotorBilda = opMode_.hardwareMap.get(DcMotorEx.class, "shooter");
        shooterMotorRev = opMode_.hardwareMap.get(DcMotorEx.class, "shooter1");
        intakeMotor = opMode_.hardwareMap.get(DcMotorEx.class, "intake");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterMotorBilda.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotorBilda.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotorRev.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotorRev.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Shooter Servos
        blockSer = opMode_.hardwareMap.get(Servo.class, "blockSer");
        closeServo();

        hoodSer = opMode_.hardwareMap.get(Servo.class, "hoodSer");
        ballStop = opMode_.hardwareMap.get(Servo.class, "ballstop");

        // Sensors
        BB0 = opMode_.hardwareMap.get(DigitalChannel.class, "BB0");
        BB1 = opMode_.hardwareMap.get(DigitalChannel.class, "BB1");
        BB2 = opMode_.hardwareMap.get(DigitalChannel.class, "BB2");
        BB3 = opMode_.hardwareMap.get(DigitalChannel.class, "BB3");
        gobuildaBB = opMode_.hardwareMap.get(DigitalChannel.class, "gobuildaBB");
        gobuildaBB1 = opMode_.hardwareMap.get(DigitalChannel.class, "gobuildaBB1");

        try {
            beamBreak = opMode_.hardwareMap.get(DigitalChannel.class, "distanceSensor");
        } catch (Exception e) {
            beamBreak = null;
        }

        shooterCoefficientsBilda = shooterMotorBilda.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterCoefficientsRev = shooterMotorRev.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        newShooterCoefficients = new PIDFCoefficients(1.0, 0.0, 0.0, 30.0);
        shooterMotorBilda.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newShooterCoefficients);
        shooterMotorRev.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newShooterCoefficients);


        /* --- LIMELIGHT INIT --- */
        try {
            limelight = opMode_.hardwareMap.get(Limelight3A.class, "limelight");
            limelight.start();
            limelight.pipelineSwitch(0);
        } catch (Exception e) {
            limelight = null;
        }
    }

    /**
     * Secondary Auto Constructor
     */
    public RobotHardware(LinearOpMode opMode, boolean auto) {
        this(opMode);
        velThresh = 1;
    }


    /* =========================================
     * DRIVETRAIN & ODOMETRY METHODS
     * ========================================= */

    private void resetOdometry() {
        for (DcMotorEx odo : new DcMotorEx[]{leftOdo, rightOdo, strafeOdo}) {
            odo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            odo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void powerMotors(double leftFront, double leftBack, double rightBack, double rightFront) {
        leftFrontDrive.setPower(leftFront);
        leftBackDrive.setPower(leftBack);
        rightFrontDrive.setPower(rightFront);
        rightBackDrive.setPower(rightBack);
    }

    public void addDrivePower(double leftFront, double leftBack, double rightBack, double rightFront) {
        lFDriveSpeed = leftFront;
        lBDriveSpeed = leftBack;
        rFDriveSpeed = rightFront;
        rBDriveSpeed = rightBack;

        lFDriveSpeed = Math.max(lFDriveSpeed, -1);
        lBDriveSpeed = Math.max(lBDriveSpeed, -1);
        rFDriveSpeed = Math.max(rFDriveSpeed, -1);
        rBDriveSpeed = Math.max(rBDriveSpeed, -1);
        calculateDrive();
    }

    public void addAlignPower(double leftFront, double leftBack, double rightBack, double rightFront) {
        lFAlignSpeed = leftFront;
        lBAlignSpeed = leftBack;
        rFAlignSpeed = rightFront;
        rBAlignSpeed = rightBack;

        lFAlignSpeed = Math.max(lFAlignSpeed, -1);
        lBAlignSpeed = Math.max(lBAlignSpeed, -1);
        rFAlignSpeed = Math.max(rFAlignSpeed, -1);
        rBAlignSpeed = Math.max(rBAlignSpeed, -1);

        calculateDrive();
    }

    public void calculateDrive() {
        double leftFront = lFAlignSpeed + lFDriveSpeed;
        double leftBack = lBAlignSpeed + lBDriveSpeed;
        double rightFront = rFAlignSpeed + rFDriveSpeed;
        double rightBack = rBAlignSpeed + rBDriveSpeed;

        if (Math.abs(leftFront) >= 1) leftFront /= Math.abs(leftFront);
        if (Math.abs(leftBack) >= 1) leftBack /= Math.abs(leftBack);
        if (Math.abs(rightFront) >= 1) rightFront /= Math.abs(rightFront);
        if (Math.abs(rightBack) >= 1) rightBack /= Math.abs(rightBack);

        powerMotors(leftFront, leftBack, rightBack, rightFront);
    }

    public void turnToFree(double degrees, double speed) {
        double oppDeg;
        double mult;
        double heading = robotHeadingAngles();

        mult = Math.abs(Math.abs(degrees) - Math.abs(heading)) * PGAIN;
        if (!(degrees - TURN_THRESH < heading && heading < degrees + TURN_THRESH)) {
            if (degrees > 0) {
                oppDeg = degrees - 180;
                if (oppDeg < heading && heading < degrees) {
                    addAlignPower(Math.min(-speed * mult, 1), Math.min(-speed * mult, 1), Math.min(speed * mult, 1), Math.min(speed * mult, 1));
                } else {
                    addAlignPower(Math.min(speed * mult, 1), Math.min(speed * mult, 1), Math.min(-speed * mult, 1), Math.min(-speed * mult, 1));
                }
            } else {
                oppDeg = degrees + 180;
                if (degrees < heading && heading < oppDeg) {
                    addAlignPower(Math.min(speed * mult, 1), Math.min(speed * mult, 1), Math.min(-speed * mult, 1), Math.min(-speed * mult, 1));
                } else {
                    addAlignPower(Math.min(-speed * mult, 1), Math.min(-speed * mult, 1), Math.min(speed * mult, 1), Math.min(speed * mult, 1));
                }
            }
        } else {
            addAlignPower(0, 0, 0, 0);
        }
    }

    public void turnTo(double degrees, double speed) {
        double oppDeg;
        double mult;
        double heading = robotHeadingAngles();

        mult = Math.abs(Math.abs(degrees) - Math.abs(heading)) * PGAIN;
        while (!(degrees - TURN_THRESH < heading && heading < degrees + TURN_THRESH) && opMode_.opModeIsActive()) {
            heading = robotHeadingAngles();
            if (degrees > 0) {
                oppDeg = degrees - 180;
                if (oppDeg < heading && heading < degrees) {
                    addAlignPower(Math.min(-speed * mult, 1), Math.min(-speed * mult, 1), Math.min(speed * mult, 1), Math.min(speed * mult, 1));
                } else if (degrees < heading && heading < oppDeg) {
                    addAlignPower(Math.min(speed * mult, 1), Math.min(speed * mult, 1), Math.min(-speed * mult, 1), Math.min(-speed * mult, 1));
                } else {
                    addAlignPower(0, 0, 0, 0);
                }
            } else {
                oppDeg = degrees + 180;
                if (degrees < heading && heading < oppDeg) {
                    addAlignPower(Math.min(speed * mult, 1), Math.min(speed * mult, 1), Math.min(-speed * mult, 1), Math.min(-speed * mult, 1));
                } else if (oppDeg < heading && heading < degrees) {
                    addAlignPower(Math.min(-speed * mult, 1), Math.min(-speed * mult, 1), Math.min(speed * mult, 1), Math.min(speed * mult, 1));
                } else {
                    addAlignPower(0, 0, 0, 0);
                }
            }
        }
        addAlignPower(0, 0, 0, 0);
    }

    public void turnToEstimate(boolean red) {
        double degrees = red ? -45 : 45;
        turnToFree(degrees, 0.5);
    }

    public double robotHeadingRadians() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public double robotHeadingAngles() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void resetImu() {
        imu.resetYaw();
        if (follower != null) {
            Pose p = follower.getPose();
            follower.setPose(new Pose(p.getX(), p.getY(), 0));
        }
    }


    /* =========================================
     * SHOOTER, INTAKE & FLICK METHODS
     * ========================================= */

    public void intake(double power) {
        double temp = Math.max(-1.0, Math.min(1.0, power));
        intakeMotor.setPower(temp);
    }

    public void setShootPower(double power) {
        double temp = Math.max(-1.0, Math.min(1.0, power));
        shooterMotorBilda.setPower(temp);
        shooterMotorRev.setPower(temp);
    }

    public void feed() {
        intake(1);
        openServo();
    }

    public void extake(double power) {
        intake(-power);
        shooterMotorBilda.setPower(-power);
        shooterMotorRev.setPower(-power);
        openServo();
    }

    public void stopFeed() {
        intake(0.0);
        closeServo();
    }

    public void stopShooter() {
        shooterMotorBilda.setVelocity(0);
        shooterMotorRev.setVelocity(0);
        stopFeed();
    }

    public double getShootVelocity() {
        return (shooterMotorBilda.getVelocity() / ENCODER_TICS);
    }

    public void setShootVelocity(double velocity) {
        shooterMotorBilda.setVelocity(velocity * ENCODER_TICS);
        shooterMotorRev.setVelocity((velocity * ENCODER_TICS));
    }

    public void setShooterRPM(double rpm) {
        shooterMotorBilda.setVelocity((rpm * ENCODER_TICS) / 60.0);
        shooterMotorRev.setVelocity((rpm * ENCODER_TICS) / 60.0);
    }

    public double calculateRegression() {
        double shooterVel = lastKnownSpeed();
        double x = getBotDis();
        if (x == -999) return shooterVel;

        // Quartic regression formula from provided logic
        shooterVel = (0.0000787632 * Math.pow(x, 4)) -
                (0.0228824    * Math.pow(x, 3)) +
                (2.35453      * Math.pow(x, 2)) -
                (96.04063     * x) +
                3742.94414;

        return shooterVel;
    }

    public double getShootPowerLINREG(double vel) {
        return ((0.012034 * vel) - 0.00615);
    }

    public double lastKnownSpeed() {
        return shooterMotorBilda.getVelocity() / ENCODER_TICS;
    }

    public double getHoodAim(double distance) {
        if (distance != -999) {
            hoodAim = ((.0038895 * distance) + -0.111817);
            return hoodAim;
        } else {
            return 0.0;
        }
    }

    public double lastKnownAim() {
        return hoodAim;
    }

    public boolean withinVel(double targVel) {
        return ((targVel) < getShootVelocity()) && (getShootVelocity() < (targVel + velThresh));
    }

    public boolean belowVel(double targVel) {
        return (getShootVelocity() < (targVel - VEL_BOTTOM_THRESH));
    }


    /* =========================================
     * SERVO & SENSOR METHODS
     * ========================================= */

    public void openServo() {
        blockSer.setPosition(IN_POS);
        servoClosed = false;
    }

    public void closeServo() {
        blockSer.setPosition(OUT_POS);
        servoClosed = true;
    }

    public void stopBallHold() {
        ballStop.setPosition(ghettoIn);
    }

    public void stopBallRelease() {
        ballStop.setPosition(ghettoOut);
    }

    public void testServo(double pos) {
        blockSer.setPosition(pos);
    }

    public boolean servoClosed() {
        return servoClosed;
    }

    public void aimHood(double pos) {
        hoodSer.setPosition(pos);
    }

    public void setTimerLight(double color) {
        timerLight.setPosition(color);
    }

    public void setFullLight(double color) {
        fullLight.setPosition(color);
    }

    public boolean isBroken() {
        return beamBreak != null && beamBreak.getState();
    }

    public boolean oneBall() {
        return ((!BB0.getState() || !BB1.getState()) && gobuildaBB1.getState()) || (!BB2.getState() || !BB3.getState()) || gobuildaBB.getState();
    }

    public boolean twoBall() {
        boolean spot1 = ((!BB0.getState() || !BB1.getState()) && gobuildaBB1.getState());
        boolean spot2 = (!BB2.getState() || !BB3.getState());
        boolean spot3 = gobuildaBB.getState();
        return (spot1 && spot2) || (spot1 && spot3) || (spot2 && spot3);
    }

    public boolean threeBall() {
        return ((!BB0.getState() || !BB1.getState()) && gobuildaBB1.getState()) && (!BB2.getState() || !BB3.getState()) && gobuildaBB.getState();
    }


    /* =========================================
     * LIMELIGHT METHODS
     * ========================================= */

    public void innit(int pipeline) {
        if (limelight != null) {
            limelight.pipelineSwitch(pipeline);
            limelight.start();
            pipelineCalled = true;
        }
    }

    public int getPipeline() {
        return limelight != null ? limelight.getStatus().getPipelineIndex() : -1;
    }

    public LLResult getLatestResult() {
        return limelight != null ? limelight.getLatestResult() : null;
    }

    public void updateResult() {
        if (limelight != null) {
            llResult = limelight.getLatestResult();
        }
    }

    public void updateIMU(double IMUHeading) {
        if (limelight != null) limelight.updateRobotOrientation(IMUHeading);
    }

    public void alignWithLimelight(double speed) {
        double tx = getTx();
        if (tx == -999) {
            addAlignPower(0, 0, 0, 0);
            return;
        }
        double power = -(tx * PanelsConfig.LLPGAIN * speed);
        addAlignPower(-power, -power, power, power);
    }

    public void alignFree(double llx) {
        if (llx != -999) {
            turnToFree(robotHeadingAngles() - llx, .6);
        }
    }

    public void align(double llx) {
        if (llx != -999) {
            turnTo(robotHeadingAngles() - llx, .6);
        }
    }

    public double getBotY() {
        updateResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose();
            return botPose.getPosition().y * 254;
        }
        return -999;
    }

    public double getBotX() {
        updateResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose();
            return botPose.getPosition().x * 254;
        }
        return -999;
    }

    public double getBotZ() {
        updateResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose();
            return botPose.getPosition().z * 254;
        }
        return -999;
    }

    public double getBotDis() {
        updateResult();
        if (llResult != null && llResult.isValid()) {
            // Reverted back to the newer distance calculation
            return 68.86747 * Math.pow(llResult.getTa(), -0.5169279);
        }
        return -999;
    }

    public double getBotRot() {
        updateResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose();
            return botPose.getPosition().z;
        }
        return -999;
    }

    public double getBotCamZ() {
        updateResult();
        if (llResult != null && llResult.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
            return ((fiducials.get(0).getCameraPoseTargetSpace().getPosition().z * 100) / 2.54) * -1.635;
        }
        return -999;
    }

    public double getBotCamY() {
        updateResult();
        if (llResult != null && llResult.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
            return ((fiducials.get(0).getCameraPoseTargetSpace().getPosition().y * 100) / 2.54) * -1.635;
        }
        return -999;
    }

    public double getBotCamX() {
        updateResult();
        if (llResult != null && llResult.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
            return ((fiducials.get(0).getCameraPoseTargetSpace().getPosition().x * 100) / 2.54) * -1.635;
        }
        return -999;
    }

    public double getTy() {
        updateResult();
        if (llResult != null && llResult.isValid()) {
            return llResult.getTy();
        }
        return -999;
    }

    public double getTx() {
        updateResult();
        if (llResult != null && llResult.isValid()) {
            return -llResult.getTx(); // Negated to match new file's logic
        }
        return -999;
    }

    public double getTarea() {
        updateResult();
        if (llResult != null && llResult.isValid()) {
            return llResult.getTa();
        }
        return -999;
    }

    /* =========================================
     * MISC / UTILITIES
     * ========================================= */

    public Follower getFollower() {
        return follower;
    }
}