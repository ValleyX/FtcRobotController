package org.firstinspires.ftc.team12841;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team12841.configs.TeleOpConfig;
import org.firstinspires.ftc.team12841.pedroPathing.Constants;
import com.pedropathing.follower.Follower;

public class RobotHardware {
    public final OpMode opMode;               // reference to the running OpMode
    private final ElapsedTime runtime = new ElapsedTime(); // timing helper

    // ------------------- DRIVE MOTORS (MECANUM) -------------------
    public DcMotorEx rfMotor, rbMotor, lfMotor, lbMotor;

    // SHOOTER MOTOR
    public DcMotorEx shooterMotor;

    // ------------------- ODOMETRY ENCODERS ------------------------
    // These are expanded from *extra* motor ports used as dead wheels
    public DcMotorEx leftOdo, rightOdo, strafeOdo;

    // ------------------- SERVO HARDWARE ---------------------------
    public Servo turntableServo, shooterServo;

    // ------------------- SENSORS (IMU + LIMELIGHT) ----------------
    public IMU imu;
    public Limelight3A limelight;

    // ------------------- PEDRO FOLLOWER (PATHING ENGINE) ----------
    private Follower follower;

    // ------------------- DRIVE POWER ACCUMULATORS -----------------
    // The code supports "drive power" + "alignment power" added together
    public double lFDriveSpeed = 0.0;
    public double rFDriveSpeed = 0.0;
    public double lBDriveSpeed = 0.0;
    public double rBDriveSpeed = 0.0;

    public double lFAlignSpeed = 0.0;
    public double rFAlignSpeed = 0.0;
    public double lBAlignSpeed = 0.0;
    public double rBAlignSpeed = 0.0;

    // ==============================================================
    // CONSTRUCTOR — initializes ALL hardware from the OpMode
    // ==============================================================
    public RobotHardware(OpMode opMode) {
        this.opMode = opMode;

        // ---------------- MOTOR + ODOMETRY INITIALIZATION ----------
        try {
            rfMotor = opMode.hardwareMap.get(DcMotorEx.class, "rfMotor");
            rbMotor = opMode.hardwareMap.get(DcMotorEx.class, "rbMotor");
            lfMotor = opMode.hardwareMap.get(DcMotorEx.class, "lfMotor");
            lbMotor = opMode.hardwareMap.get(DcMotorEx.class, "lbMotor");

            shooterMotor = opMode.hardwareMap.get(DcMotorEx.class, "shooterMotor");

            // dead-wheel odometry encoders
            leftOdo   = opMode.hardwareMap.get(DcMotorEx.class, "leftOdo");
            rightOdo  = opMode.hardwareMap.get(DcMotorEx.class, "rightOdo");
            strafeOdo = opMode.hardwareMap.get(DcMotorEx.class, "strafeOdo");

        } catch (Exception e) {
            opMode.telemetry.addLine("ERROR: Motor/Odo mapping failed!");
        }

        // ---------------- SERVO INITIALIZATION ---------------------
        try {
            shooterServo  = opMode.hardwareMap.get(Servo.class, "shooterServo");
            turntableServo = opMode.hardwareMap.get(Servo.class, "turntableServo");
        } catch (Exception e) {
            opMode.telemetry.addLine("ERROR: Servo mapping failed!");
        }

        // ---------------- IMU INITIALIZATION -----------------------
        try {
            imu = opMode.hardwareMap.get(IMU.class, "imu");

            // Orientation settings: logo RIGHT, USB UP on the control hub
            imu.initialize(new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )
            ));

        } catch (Exception e) {
            opMode.telemetry.addLine("ERROR: IMU failed!");
        }

        // ---------------- LIMELIGHT INITIALIZATION -----------------
        try {
            limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0);   // default pipeline
            limelight.start();             // begin image streaming
            opMode.telemetry.addLine("Limelight3A ONLINE");
        } catch (Exception e) {
            limelight = null;
            opMode.telemetry.addLine("Limelight3A NOT FOUND");
        }

        // ---------------- MOTOR CONFIGURATION ----------------------
        try {
            // Reverse left motors to match orientation of mecanum drive
            lfMotor.setDirection(DcMotor.Direction.REVERSE);
            lbMotor.setDirection(DcMotor.Direction.REVERSE);

            rfMotor.setDirection(DcMotor.Direction.FORWARD);
            rbMotor.setDirection(DcMotor.Direction.FORWARD);

            // Run without encoder so Pedro can apply custom kinematics
            for (DcMotorEx m : new DcMotorEx[]{lfMotor, lbMotor, rfMotor, rbMotor}) {
                m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            // Shooter should keep its encoder enabled
            shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Reset odometry encoders before use
            for (DcMotorEx odo : new DcMotorEx[]{leftOdo, rightOdo, strafeOdo}) {
                odo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                odo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                odo.setPower(0);
            }

        } catch (Exception e) {
            opMode.telemetry.addLine("ERROR: motor/encoder config failed!");
        }

        // ---------------- PEDRO FOLLOWER ---------------------------
        try {
            // Create the path follower from Pedro constants
            follower = Constants.createFollower(opMode.hardwareMap);
            opMode.telemetry.addLine("Pedro Pathing OK");
        } catch (Exception e) {
            follower = null;
            opMode.telemetry.addLine("Pedro FAILED");
        }

        opMode.telemetry.update();
    }

    // ---------------- INTERNAL LIMELIGHT STATE --------------------
    private LinearOpMode opMode_;
    LLResult llResult;
    boolean pipelineCalled = false;

    // ===================== LIMELIGHT HELPERS ======================
    public void innit(int pipeline){
        // Switch LL to a requested pipeline index
        limelight.pipelineSwitch(pipeline);
        limelight.start();
        pipelineCalled = true;
    }

    public int getPipeline(){
        return limelight.getStatus().getPipelineIndex();
    }

    public LLResult getLatestResult(){
        return limelight.getLatestResult();
    }

    // Refresh LL data every call
    public void updateResult(){
        llResult = limelight.getLatestResult();
    }

    // IMU heading forwarded to LL for robot-centric odometry fusion
    public void updateIMU(double IMUHeading){
        limelight.updateRobotOrientation(IMUHeading);
    }

    // Distance estimation from LL target area (ta) and regression equation
    public double getBotDis(){
        updateResult();
        if (llResult != null && llResult.isValid()) {
            return 68.86747*Math.pow(llResult.getTa(), -0.5169279);
        }
        return -999; // invalid
    }

    // Horizontal offset (tx) from LL; main value used for alignment
    public double getTx(){
        updateResult();
        if (llResult != null && llResult.isValid()) {
            return llResult.getTx();
        }
        return -999; // invalid reading
    }

    // Return Pedro follower instance
    public Follower getFollower() {
        return follower;
    }

    public void resetIMU() {
        if (imu != null) imu.resetYaw();
    }

    // ==================== DRIVE / ALIGNMENT LOGIC =================
    // “Carter’s Stuff”: custom tuned drive math + IMU + LL turning

    public void powerMotors(double leftFront, double leftBack, double rightBack, double rightFront){
        // Direct motor power command (no normalization)
        lfMotor.setPower(leftFront);
        lbMotor.setPower(leftBack);
        rfMotor.setPower(rightFront);
        rbMotor.setPower(rightBack);
    }

    public void addDrivePower(double leftFront, double leftBack, double rightBack, double rightFront){
        // Accumulates *drive* values
        lFDriveSpeed = leftFront;
        lBDriveSpeed = leftBack;
        rFDriveSpeed = rightFront;
        rBDriveSpeed = rightBack;

        // Normalize if needed
        if(Math.abs(lFDriveSpeed) >= 1) lFDriveSpeed /= Math.abs(lFDriveSpeed);
        if(Math.abs(lBDriveSpeed) >= 1) lBDriveSpeed /= Math.abs(lBDriveSpeed);
        if(Math.abs(rFDriveSpeed) >= 1) rFDriveSpeed /= Math.abs(rFDriveSpeed);
        if(Math.abs(rBDriveSpeed) >= 1) rBDriveSpeed /= Math.abs(rBDriveSpeed);

        calculateDrive();
    }

    public void addAlignPower(double leftFront, double leftBack, double rightBack, double rightFront){
        // Accumulates *alignment* values
        lFAlignSpeed = leftFront;
        lBAlignSpeed = leftBack;
        rFAlignSpeed = rightFront;
        rBAlignSpeed = rightBack;

        // Normalize if needed
        if(Math.abs(lFAlignSpeed) >= 1) lFAlignSpeed /= Math.abs(lFAlignSpeed);
        if(Math.abs(lBAlignSpeed) >= 1) lBAlignSpeed /= Math.abs(lBAlignSpeed);
        if(Math.abs(rFAlignSpeed) >= 1) rFAlignSpeed /= Math.abs(rFAlignSpeed);
        if(Math.abs(rBAlignSpeed) >= 1) rBAlignSpeed /= Math.abs(rBAlignSpeed);

        calculateDrive();
    }

    // IMU-based turn controller (open-loop P + region checks)
    public void turnToFree(double degrees, double speed){
        double oppDeg;
        double mult;
        double heading = robotHeadingAngles();

        mult = Math.abs(Math.abs(degrees) - Math.abs(heading)) * TeleOpConfig.PGAIN;

        // while not within tolerance, compute quadrant-based turning power
        if(!( degrees - TeleOpConfig.TURN_THRESH < heading && heading < degrees + TeleOpConfig.TURN_THRESH)) {
            if (degrees > 0) {
                oppDeg = degrees - 180;
                if (oppDeg < heading && heading < degrees) {
                    addAlignPower(Math.min(-speed * mult, 1), Math.min(-speed * mult, 1),
                            Math.min(speed * mult, 1), Math.min(speed * mult, 1));
                } else if (degrees < heading && heading < oppDeg){
                    addAlignPower(Math.min(speed * mult, 1), Math.min(speed * mult, 1),
                            Math.min(-speed * mult, 1), Math.min(-speed * mult, 1));
                } else {
                    addAlignPower(0,0,0,0);
                }
            } else {
                oppDeg = degrees + 180;
                if (degrees < heading && heading < oppDeg) {
                    addAlignPower(Math.min(speed * mult, 1), Math.min(speed * mult, 1),
                            Math.min(-speed * mult, 1), Math.min(-speed * mult, 1));
                } else if (oppDeg < heading && heading < degrees) {
                    addAlignPower(Math.min(-speed * mult, 1), Math.min(-speed * mult, 1),
                            Math.min(speed * mult, 1), Math.min(speed * mult, 1));
                } else {
                    addAlignPower(0,0,0,0);
                }
            }
        } else {
            addAlignPower(0,0,0,0);
        }
    }

    // Combine drive + align and send to motors
    public void calculateDrive(){
        double leftFront  = lFAlignSpeed + lFDriveSpeed;
        double leftBack   = lBAlignSpeed + lBDriveSpeed;
        double rightFront = rFAlignSpeed + rFDriveSpeed;
        double rightBack  = rBAlignSpeed + rBDriveSpeed;

        // Normalize again
        if(Math.abs(leftFront) >= 1)  leftFront  /= Math.abs(leftFront);
        if(Math.abs(leftBack) >= 1)   leftBack   /= Math.abs(leftBack);
        if(Math.abs(rightFront) >= 1) rightFront /= Math.abs(rightFront);
        if(Math.abs(rightBack) >= 1)  rightBack  /= Math.abs(rightBack);

        powerMotors(leftFront, leftBack, rightBack, rightFront);
    }

    // IMU yaw in radians
    public double robotHeadingRadians(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    // IMU yaw in degrees
    public double robotHeadingAngles(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void resetImu(){
        imu.resetYaw();
    }

    // LL steering correction without stopping driver control
    public void alignFree(double llx) {
        if (llx != -999){
            turnToFree(robotHeadingAngles() - llx, .6);
        }
    }

    // LL steering correction with hard turnTo()
    public void align(double llx){
        if(llx != -999) {
            turnTo(robotHeadingAngles() - llx, .6);
        }
    }

    // If alliance color is known, estimate goal direction and turn toward it
    public void turnToEstimate(boolean red){
        double degrees = red ? -45 : 45;
        turnToFree(degrees, 0.5);
    }

    // Blocking rotation method (linear OpMode only)
    public void turnTo(double degrees, double speed){
        double oppDeg;
        double mult;
        double heading = robotHeadingAngles();

        mult = Math.abs(Math.abs(degrees) - Math.abs(heading)) * TeleOpConfig.PGAIN;

        // While not within tolerance, actively adjust orientation
        while(!( degrees - TeleOpConfig.TURN_THRESH < heading && heading < degrees + TeleOpConfig.TURN_THRESH) && opMode_.opModeIsActive()) {
            heading = robotHeadingAngles();
            if (degrees > 0) {
                oppDeg = degrees - 180;
                if (oppDeg < heading && heading < degrees) {
                    addAlignPower(Math.min(-speed * mult, 1), Math.min(-speed * mult, 1),
                            Math.min(speed * mult, 1), Math.min(speed * mult, 1));
                } else if (degrees < heading && heading < oppDeg){
                    addAlignPower(Math.min(speed * mult, 1), Math.min(speed * mult, 1),
                            Math.min(-speed * mult, 1), Math.min(-speed * mult, 1));
                } else {
                    addAlignPower(0,0,0,0);
                }
            } else {
                oppDeg = degrees + 180;
                if (degrees < heading && heading < oppDeg) {
                    addAlignPower(Math.min(speed * mult, 1), Math.min(speed * mult, 1),
                            Math.min(-speed * mult, 1), Math.min(-speed * mult, 1));
                } else if (oppDeg < heading && heading < degrees) {
                    addAlignPower(Math.min(-speed * mult, 1), Math.min(-speed * mult, 1),
                            Math.min(speed * mult, 1), Math.min(speed * mult, 1));
                } else {
                    addAlignPower(0,0,0,0);
                }
            }
        }
        addAlignPower(0,0,0,0);
    }

    // Full stop on all wheels
    public void applyFullBrake() {
        follower.setTeleOpDrive(0, 0, 0, false);

        lfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lfMotor.setPower(0);
        lbMotor.setPower(0);
        rfMotor.setPower(0);
        rbMotor.setPower(0);
    }

    // Reset Heading
    public void resetPedroHeading() {
        if (follower == null) return;

        // Step 1: Reset IMU orientation
        imu.resetYaw();

        // Step 2: Get the existing pose from Pedro
        Pose currentPose = follower.getPose();

        if (currentPose == null) return;

        // Step 3: Build a new pose with SAME X/Y but zeroed heading
        Pose newPose = new Pose(
                currentPose.getX(),
                currentPose.getY(),
                0 // reset heading to 0 radians
        );

        // Step 4: Apply pose to Pedro
        follower.setPose(newPose);
    }

}
