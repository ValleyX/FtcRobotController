package org.firstinspires.ftc.team12841.teleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.limelightvision.*;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.team12841.RobotHardware;

@TeleOp(name = "TeleOp Main", group = "opModes")
public class teleOpPathing extends OpMode {

    private RobotHardware robot;

    // Shooter state
    private boolean shooterActive = false;
    private boolean bPressedPrev = false;
    private double shooterStartTime = 0;

    // Turntable state
    private int currentTurntableIndex = 0;
    private boolean leftBumperPrev = false;
    private boolean rightBumperPrev = false;
    private boolean aPressedPrev = false;
    private boolean autoCycling = false;
    private double autoCycleStartTime = 0;
    private int autoCycleStage = 0;

    // Baby mode
    private boolean babyMode = false;
    private boolean babyTogglePrev = false;
    private final double BABY_MODE_SCALE = 0.4;

    // --- Servo preset positions ---
    private final double SHOOTER_IDLE = 0.15;
    private final double SHOOTER_FIRE = 0.85;
    private final double[] TURNTABLE_POSITIONS = {0.2, 0.8, 0.0};

    private final double TARGET_TAG_ID = 4;        // change to your correct tag ID
    private final double DESIRED_DISTANCE_IN = 48; // target shooting distance (inches)
    private final double AIM_KP = 0.012;           // rotation proportional gain
    private final double DRIVE_KP = 0.04;          // distance proportional gain

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        robot = new RobotHardware(this);

        // Limelight setup
        // --- Limelight 3A ---
     //   Limelight3A limelight = robot.limelight3A;
     //   if (limelight != null) {
     //       limelight.pipelineSwitch(0); // use default pipeline
     //       limelight.start();           // enable streaming
     //   }

        telemetry.addLine("TeleOp initialized with LL3A");
        telemetry.update();
    }

    @Override
    public void loop() {
        double time = runtime.seconds();

        // --- Toggle baby mode (GP1 right bumper) ---
        if (gamepad1.right_bumper && !babyTogglePrev) babyMode = !babyMode;
        babyTogglePrev = gamepad1.right_bumper;

        // --- Driving ---
        double x = gamepad1.right_stick_x;
        double y = -gamepad1.right_stick_y;
        double rot = gamepad1.left_stick_x;

        double scale = babyMode ? BABY_MODE_SCALE : 1.0;
        x *= scale;
        y *= scale;
        rot *= scale;

        if (robot.getFollower() != null)
            robot.getFollower().setTeleOpDrive(y, x, rot, true);
        else
            driveManual(y, x, rot);

        // --- Turntable bumpers (next/previous) ---
        if (gamepad2.right_bumper && !rightBumperPrev) moveTurntableNext();
        if (gamepad2.left_bumper && !leftBumperPrev) moveTurntablePrev();
        rightBumperPrev = gamepad2.right_bumper;
        leftBumperPrev = gamepad2.left_bumper;

        // --- Shooter trigger control (RT) ---
        if (gamepad2.right_trigger >= 0.8) {
            setShooterPower(1.0);          // motor full speed
            setShooterServo(SHOOTER_FIRE); // flick fire
        } else {
            setShooterServo(SHOOTER_IDLE);
            if (!shooterActive) setShooterPower(0.0);
        }

        // --- Shooter toggle with auto-off after 10s (B) ---
        if (gamepad2.b && !bPressedPrev) {
            shooterActive = !shooterActive;
            shooterStartTime = time;
        }
        bPressedPrev = gamepad2.b;

        if (shooterActive) {
            setShooterPower(1.0);
            if (time - shooterStartTime >= 10) {
                shooterActive = false;
                setShooterPower(0.0);
            }
        }

        // --- Auto cycle through turntable positions (A) ---
        if (gamepad2.a && !aPressedPrev && !autoCycling) {
            autoCycling = true;
            autoCycleStartTime = time;
            autoCycleStage = 0;
        }
        aPressedPrev = gamepad2.a;

        if (autoCycling) {
            double elapsed = time - autoCycleStartTime;
            int stage = (int)(elapsed / 1.5);
            if (stage < TURNTABLE_POSITIONS.length) {
                setTurntableServo(TURNTABLE_POSITIONS[stage]);
                currentTurntableIndex = stage;
            } else {
                autoCycling = false;
            }
        }

        // --- LL3A auto-aim ---
//        if (gamepad2.left_trigger >= 0.8 && limelight != null) {
//            Limelight3AResults results = limelight.getLatestResults();
//
 //            if (results != null && results.isValid()) {
//                TagResult tag = results.getTagById(TARGET_TAG_ID);
//
 //                if (tag != null) {
//                    double tx = tag.tx(); // horizontal offset (degrees)
//                    double tz = tag.tz(); // forward distance (inches)
//
 //                    // Proportional control
//                    double rotCmd = tx * AIM_KP;
//                    double distError = (DESIRED_DISTANCE_IN - tz);
//                    double driveCmd = distError * DRIVE_KP;
//
 //                    // Send correction commands
//                    if (robot.getFollower() != null)
//                        robot.getFollower().setTeleOpDrive(driveCmd, 0, rotCmd, true);
//                    else
//                        driveManual(driveCmd, 0, rotCmd);

//                    // Optional: blend LL pose with Pedro localization
//                    if (robot.getFollower() != null) {
//                        Pose estPose = robot.getFollower().getPose();
//                        Pose llPose = new Pose(tag.fieldX(), tag.fieldY(), tag.fieldYaw());
//                        Pose corrected = new Pose(
//                                (estPose.x * 0.9) + (llPose.x * 0.1),
//                                (estPose.y * 0.9) + (llPose.y * 0.1),
//                                estPose.heading
//                        );
//                        robot.getFollower().setPose(corrected);
//                    }

//                    telemetry.addData("LL Target", tag.getId());
//                    telemetry.addData("tx (deg)", tx);
//                    telemetry.addData("tz (in)", tz);
//                    telemetry.addData("RotCmd", rotCmd);
//                    telemetry.addData("DriveCmd", driveCmd);
//                } else {
//                    telemetry.addLine("LL3A: No tag visible");
//                }
//            } else {
//                telemetry.addLine("LL3A: No valid data");
//            }
//        }

        // --- Index correction safety (in case of manual override) ---
        if (currentTurntableIndex < 0 || currentTurntableIndex >= TURNTABLE_POSITIONS.length) {
            currentTurntableIndex = 0;
            setTurntableServo(TURNTABLE_POSITIONS[currentTurntableIndex]);
        }

        // --- Telemetry ---
        telemetry.addData("Shooter Active", shooterActive);
        telemetry.addData("Baby Mode", babyMode ? "ON" : "OFF");
        telemetry.addData("Turntable Index", currentTurntableIndex);
        telemetry.addData("Auto Cycle", autoCycling);
        telemetry.update();
    }

    // ========== CONTROL METHODS ==========

    private void setShooterPower(double power) {
        if (robot.shooterMotor != null) robot.shooterMotor.setPower(power);
    }

    private void setShooterServo(double position) {
        if (robot.shooterServo != null) robot.shooterServo.setPosition(position);
    }

    private void setTurntableServo(double position) {
        if (robot.turntableServo != null) robot.turntableServo.setPosition(position);
    }

    private void moveTurntableNext() {
        currentTurntableIndex++;
        if (currentTurntableIndex >= TURNTABLE_POSITIONS.length) currentTurntableIndex = 0;
        setTurntableServo(TURNTABLE_POSITIONS[currentTurntableIndex]);
    }

    private void moveTurntablePrev() {
        currentTurntableIndex--;
        if (currentTurntableIndex < 0) currentTurntableIndex = TURNTABLE_POSITIONS.length - 1;
        setTurntableServo(TURNTABLE_POSITIONS[currentTurntableIndex]);
    }

    private void driveManual(double y, double x, double rot) {
        double fl = y + x + rot;
        double fr = y - x - rot;
        double bl = y - x + rot;
        double br = y + x - rot;

        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        robot.lfMotor.setPower(fl / max);
        robot.rfMotor.setPower(fr / max);
        robot.lbMotor.setPower(bl / max);
        robot.rbMotor.setPower(br / max);
    }
}
