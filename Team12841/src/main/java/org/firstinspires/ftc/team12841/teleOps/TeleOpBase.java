package org.firstinspires.ftc.team12841.teleOps;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team12841.RobotHardware;
import org.firstinspires.ftc.team12841.configs.PanelsConfig;

@Disabled
@TeleOp(name = "TeleOp Base")
public class TeleOpBase extends LinearOpMode {

    /* ===================== HARDWARE ===================== */
    private RobotHardware robot;
    private Follower follower;

    /* ===================== PEDRO ===================== */
    private boolean poseReady = false;

    /* ===================== STATE ===================== */
    private double targetRPM = 1500.0;
    private double rpmOffset = 0.0;

    public boolean babyMode = false;
    public int pipeline = 0;

    // Toggles for manual adjustment and inputs
    private boolean dUp = false;
    private boolean dDown = false;
    private boolean lastStart = false;
    private boolean lastRightStick = false;

    // Timers
    private Timer timer;
    private Timer flash;
    boolean flashActive = false;
    private boolean lastBumper = false;
    private Timer shootTimer = new Timer();

    @Override
    public void runOpMode() throws InterruptedException {

        /* ===================== INIT ===================== */
        robot = new RobotHardware(this);
        follower = robot.getFollower();

        telemetry.addLine("Init OK — warming localization");
        telemetry.update();

        robot.innit(pipeline);

        timer = new Timer();
        flash = new Timer();

        /* ===================== INIT LOOP ===================== */
        while (opModeInInit()) {
            if (follower != null) {
                follower.update();
            }

            if (follower != null && follower.getPose() != null) {
                poseReady = true;
                telemetry.addLine("POSE READY");
            } else {
                telemetry.addLine("Warming localization...");
            }

            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        /* ===================== START ===================== */
        if (follower != null) {
            follower.startTeleopDrive(true);
            follower.update();
            timer.resetTimer();
        }
        robot.stopBallHold();

        /* ===================== MAIN LOOP ===================== */
        while (opModeIsActive()) {

            /* ---------- SAFETY ---------- */
            if (follower == null || !poseReady || follower.getPose() == null) {
                telemetry.addLine("Drive unavailable");
                telemetry.update();
                continue;
            }

            follower.update();

            /* ---------- DRIVE INPUTS ---------- */
            double rotate = -gamepad1.right_stick_x;
            double strafe = -gamepad1.left_stick_x;
            double forward = -gamepad1.left_stick_y;

            boolean currentStart = gamepad1.start;
            boolean currentRightStick = gamepad1.right_stick_button;

            if ((currentStart && !lastStart) || (currentRightStick && !lastRightStick)) {
                babyMode = !babyMode;
            }
            lastStart = currentStart;
            lastRightStick = currentRightStick;

            /* ---------- LIMELIGHT ALIGN (RIGHT BUMPER) ---------- */
            if (gamepad1.right_bumper) {
                robot.alignWithLimelight(-1);
            } else if (babyMode) {
                follower.setTeleOpDrive(
                        forward * PanelsConfig.BABY,
                        strafe * PanelsConfig.BABY,
                        rotate * PanelsConfig.BABY,
                        false
                );
            } else {
                follower.setTeleOpDrive(forward, strafe, rotate, false);
            }

            if (gamepad1.guide) {
                robot.resetImu();
            }

            /* ---------- INTAKE & EXTAKE ---------- */
            boolean isFull = robot.threeBall();
            robot.setFullLight(isFull ? 1.0 : 0.0);

            boolean intaking = false;

            if (gamepad1.right_trigger > 0.2) {
                // Intake
                robot.intake(1.0);
                intaking = true;
            } else if (gamepad1.left_trigger > 0.2) {
                // Extake
                robot.extake(1.0);
                robot.closeServo(); // Assuming this is needed for extake based on original code
                intaking = true;
            } else {
                if (!gamepad1.right_bumper) {
                    robot.intake(0);
                }
            }

            /* ---------- CONTINUOUS SHOOTER & HOOD ---------- */
            // Manual RPM Offset
            if (gamepad1.dpad_up) {
                if (!dUp) { targetRPM += 100; dUp = true; }
            } else { dUp = false; }

            if (gamepad1.dpad_down) {
                if (!dDown) { targetRPM -= 100; dDown = true; }
            } else { dDown = false; }

            // Constantly update Hood and RPM based on distance
            if(robot.getTx() != -999)
            {
                //targetRPM = robot.calculateRegression();
            }

            robot.setShooterRPM(targetRPM);
            //robot.shooterMotor.setPower(1);

            /* ---------- SHOOTING LOGIC (RIGHT BUMPER) ---------- */
            if (gamepad1.right_bumper) {
                // 1. On the very first press, reset the timer
                if (!lastBumper) {
                    shootTimer.resetTimer();
                    robot.stopBallRelease(); // Open the blocker immediately
                }

                // 2. Wait for 300ms, then check velocity to feed
                if (shootTimer.getElapsedTimeSeconds() > 0.3) {
                    robot.aimHood(robot.getHoodAim(robot.getBotDis()));
                    robot.feed();
                }
            } else {
                // Reset state when bumper is released
                robot.stopBallHold();
                if (!intaking) {
                    robot.stopFeed();
                }
            }
            lastBumper = gamepad1.right_bumper; // Track state for the next frame

            /* ---------- LIGHT TIMERS ---------- */
            double t = timer.getElapsedTimeSeconds();

            if (t >= 100) {
                if (!flashActive) {
                    flash.resetTimer();
                    flashActive = true;
                }
                double ft = flash.getElapsedTimeSeconds();
                if (ft < 0.333) {
                    robot.setTimerLight(0.27);
                } else if (ft < 0.667) {
                    robot.setTimerLight(0.0);
                } else if (ft < 1.0) {
                    robot.setTimerLight(0.27);
                } else {
                    flash.resetTimer();
                }

            } else if (t >= 80) {
                robot.setTimerLight(0.33);
                flashActive = false;

            } else if (t >= 60) {
                if (!flashActive) {
                    flash.resetTimer();
                    flashActive = true;
                }
                double ft = flash.getElapsedTimeSeconds();
                if (ft < 0.333) {
                    robot.setTimerLight(0.39);
                } else if (ft < 0.667) {
                    robot.setTimerLight(0.0);
                } else if (ft < 1.0) {
                    robot.setTimerLight(0.39);
                } else {
                    flash.resetTimer();
                }
            } else {
                robot.setTimerLight(0.5);
                flashActive = false;
            }

            /* ---------- TELEMETRY ---------- */
            double actualRPM = (robot.shooterMotorBilda.getVelocity() * 60.0) / robot.ENCODER_TICS;

            telemetry.addData("--- DRIVE STATE ---", "");
            telemetry.addData("BabyMode", babyMode);
            telemetry.addData("IMU (Degrees)", robot.robotHeadingAngles());

            telemetry.addData("--- SHOOTER ---", "");
            telemetry.addData("Target RPM (incl. offset)", targetRPM);
            telemetry.addData("Actual RPM", actualRPM);
            telemetry.addData("Servo Closed?", robot.servoClosed());

            telemetry.addData("--- INTAKE ---", "");
            telemetry.addData("Three Balls (Full)", isFull);
            telemetry.addData("At least One Ball", robot.oneBall());
            telemetry.addData("At least Two Balls", robot.twoBall());

            telemetry.addData("--- VISION ---", "");
            telemetry.addData("LL Distance", robot.getBotDis());
            telemetry.addData("LL Tx", robot.getTx());

            telemetry.addData("--- HARDWARE ---", "");
            telemetry.addData("Batt Volt", "%.1f", robot.batteryVoltSensor.getVoltage());
            telemetry.update();
        }
    }
}