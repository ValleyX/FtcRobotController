package org.firstinspires.ftc.team12841.teleOps;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.team12841.RobotHardware;
import org.firstinspires.ftc.team12841.configs.TeleOpConfig;

@TeleOp(name = "TeleOp Main", group = "opModes")
public class TeleOpTwoPeople extends OpMode {

    // ============================================================
    // CORE ROBOT + FOLLOWER
    // ============================================================
    private RobotHardware robot;    // All motors, servos, sensors, LL, IMU, follower wrapper
    private Follower follower;       // Pedro teleOp localization + field-centric drive
    private boolean poseReady = false; // Ensures Pedro pose is initialized before driving


    // ============================================================
    // SHOOTER VARIABLES
    // ============================================================
    public boolean startShooter = false; // Toggles shooter motor on/off
    private boolean bPrev = false;       // Debounce for gamepad2 B button
    public double shootPwr = 1.0;        // Shooter motor power level

    private boolean DpadDownPrev = false; // Debounce for down adjustment
    private boolean DpadUpPrev   = false; // Debounce for up adjustment


    // ============================================================
    // DRIVE MODES
    // ============================================================
    private boolean baby = false;     // Slow mode (higher precision)
    private boolean babyPrev = false;

    private boolean oldMan = false;   // VERY slow mode (super controlled)
    private boolean oldManPrev = false;


    // ============================================================
    // TURNTABLE / LOADING SYSTEM
    // ============================================================
    private int ttIndex = 0;           // 0 → 1 → 2 indexing for turntable
    private boolean rbPrev = false, lbPrev = false;

    private boolean aPrev = false;     // Debounce for auto-cycle
    private boolean autoCycle = false; // Auto-index turntable positions 0 → 1 → 2
    private double autoCycleStart = 0;

    private final ElapsedTime runtime = new ElapsedTime();


    // ============================================================
    // INIT
    // ============================================================
    @Override
    public void init() {
        robot = new RobotHardware(this);         // Initialize full robot hardware
        follower = robot.getFollower();          // Grab Pedro follower instance

        robot.innit(0); // Set Limelight pipeline → 0 (default scoring pipeline)

        telemetry.addLine("Init OK — LL pipeline 0");
        telemetry.update();
    }


    // ============================================================
    // INIT LOOP (runs repeatedly before start)
    // Ensures Pedro has a valid pose before teleOp begins
    // ============================================================
    @Override
    public void init_loop() {
        if (follower != null) follower.update();

        if (follower != null && follower.getPose() != null) {
            poseReady = true;                // We can drive field-centric safely
            telemetry.addLine("POSE READY");
        } else {
            telemetry.addLine("Warming localization…");
        }
        telemetry.update();
    }


    // ============================================================
    // START (called once when PLAY is pressed)
    // ============================================================
    @Override
    public void start() {
        if (follower != null) {
            follower.startTeleopDrive(); // Switch Pedro into teleOp drive mode
            follower.update();
        }
    }


    // ============================================================
    // MAIN TELEOP LOOP
    // ============================================================
    @Override
    public void loop() {

        // If Pedro is unavailable → drive manually
        if (follower == null) {
            driveManual();
            telemetry.addLine("FOLLOWER NULL — manual only");
            telemetry.update();
            return;
        }

        // If pose isn't ready → DO NOT DRIVE
        if (!poseReady) {
            follower.update();
            telemetry.addLine("Waiting for pose…");
            telemetry.update();
            return;
        }

        follower.update();

        // If Pedro loses pose mid-match → safe fallback
        if (follower.getPose() == null) {
            telemetry.addLine("POSE LOST");
            telemetry.update();
            return;
        }

        // ============================================================
        // DRIVER INPUT (GAMEPAD 1)
        // ============================================================
        double x = -gamepad1.left_stick_x;   // Strafe
        double y = -gamepad1.left_stick_y;   // Forward/back
        double r = -gamepad1.right_stick_x;  // Rotation

        // Speed profiles
        if (baby && !oldMan) {
            x *= TeleOpConfig.BABY_MODE_SCALE;
            y *= TeleOpConfig.BABY_MODE_SCALE;
            r *= TeleOpConfig.BABY_MODE_SCALE;
        }

        if (oldMan && !baby) {
            x *= TeleOpConfig.OLDMAN_MODE_SCALE;
            y *= TeleOpConfig.OLDMAN_MODE_SCALE;
            r *= TeleOpConfig.OLDMAN_MODE_SCALE;
        }

        // Limelight A-Mode rotation assistance
        double rotCmd = getRotCmd(r);

        // Apply final Pedro drive command
        follower.setTeleOpDrive(y, x, rotCmd, false);


        // ============================================================
        // AUTO BRAKE — stops drift when sticks released
        // ============================================================
        if (Math.abs(x) < 0.02 && Math.abs(y) < 0.02 && Math.abs(r) < 0.02 &&
                gamepad1.left_trigger < 0.8) {
            robot.applyFullBrake();
        }


        // ============================================================
        // EXTAKE (reverse shooter motor)
        // ============================================================
        if (gamepad2.x)
            robot.shooterMotor.setPower(-0.3);


        // ============================================================
        // BABY MODE TOGGLE (gamepad1 RB)
        // ============================================================
        if (gamepad1.right_bumper && !babyPrev) baby = !baby;
        babyPrev = gamepad1.right_bumper;


        // ============================================================
        // ANCIENT MODE TOGGLE (gamepad1 LB)
        // ============================================================
        if (gamepad1.left_bumper && !oldManPrev) oldMan = !oldMan;
        oldManPrev = gamepad1.left_bumper;


        // ============================================================
        // SHOOTER SERVO (manual fire)
        // ============================================================
        if (gamepad2.right_trigger >= 0.8)
            robot.shooterServo.setPosition(TeleOpConfig.SHOOTER_FIRE);
        else
            robot.shooterServo.setPosition(TeleOpConfig.SHOOTER_IDLE);


        // ============================================================
        // SHOOTER POWER ADJUSTMENTS
        // ============================================================
        if (gamepad2.dpad_down && !DpadDownPrev) {
            shootPwr -= 0.025;
            if (shootPwr < 0) shootPwr = 0;
        }
        DpadDownPrev = gamepad2.dpad_down;

        if (gamepad2.dpad_up && !DpadUpPrev) {
            shootPwr += 0.025;
            if (shootPwr > 1) shootPwr = 1;
        }
        DpadUpPrev = gamepad2.dpad_up;


        // Quick presets
        if (gamepad2.dpad_right) shootPwr = 0.675;
        if (gamepad2.dpad_left)  shootPwr = 0.75;


        // ============================================================
        // SHOOTER MOTOR TOGGLE (gamepad2 B)
        // ============================================================
        if (gamepad2.b && !bPrev) startShooter = !startShooter;
        bPrev = gamepad2.b;

        robot.shooterMotor.setPower(startShooter ? shootPwr : 0);


        // ============================================================
        // TURNTABLE CONTROL (positions 0 → 1 → 2)
        // ============================================================
        double[] tt = {TeleOpConfig.TT_POS_0, TeleOpConfig.TT_POS_1, TeleOpConfig.TT_POS_2};

        if (gamepad2.right_bumper && !rbPrev &&
                robot.shooterServo.getPosition() == TeleOpConfig.SHOOTER_IDLE) {
            ttIndex = (ttIndex + 1) % tt.length;
            robot.turntableServo.setPosition(tt[ttIndex]);
        }
        rbPrev = gamepad2.right_bumper;

        if (gamepad2.left_bumper && !lbPrev &&
                robot.shooterServo.getPosition() == TeleOpConfig.SHOOTER_IDLE) {
            ttIndex--;
            if (ttIndex < 0) ttIndex = tt.length - 1;
            robot.turntableServo.setPosition(tt[ttIndex]);
        }
        lbPrev = gamepad2.left_bumper;


        // ============================================================
        // AUTO LOAD CYCLE (cycles TT positions every 1.5 sec)
        // ============================================================
        if (gamepad2.a && !aPrev && !autoCycle) {
            autoCycle = true;
            autoCycleStart = runtime.seconds();
        }
        aPrev = gamepad2.a;

        if (autoCycle) {
            int stage = (int)((runtime.seconds() - autoCycleStart) / 1.5);
            if (stage < tt.length) {
                ttIndex = stage;
                robot.turntableServo.setPosition(tt[stage]);
            } else {
                autoCycle = false;
            }
        }


        // ============================================================
        // RESET IMU + PEDRO HEADING (gamepad1 GUIDE button)
        // ============================================================
        if (gamepad1.guide)
            robot.resetPedroHeading();


        // ============================================================
        // TELEMETRY
        // ============================================================
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("LL Aligning", gamepad1.left_trigger > 0.8);
        telemetry.addData("Shoot Power", shootPwr);
        telemetry.addData("Shooter Motor", startShooter);
        telemetry.addData("Baby Mode", baby);
        telemetry.addData("Ancient Mode", oldMan);
        telemetry.addData("Turntable Pos", ttIndex);
        telemetry.update();
    }


    // ============================================================
    // LIMELIGHT ROTATION ASSIST
    // ============================================================
    private double getRotCmd(double r) {
        double rotCmd = r;

        if (gamepad1.left_trigger > 0.8) {

            double tx = robot.getTx(); // Horizontal offset from target

            if (tx != -999) {
                double Kp = TeleOpConfig.KP;
                double turn = -(tx * Kp); // Negative ensures correct steering direction

                double maxTurn = 0.16;
                if (turn > maxTurn)  turn = maxTurn;
                if (turn < -maxTurn) turn = -maxTurn;

                rotCmd += turn; // Blend LL turn correction with driver turn input
            }
        }
        return rotCmd;
    }


    // ============================================================
    // MANUAL DRIVE (fallback if Pedro dies)
    // ============================================================
    private void driveManual() {
        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double r = -gamepad1.right_stick_x;

        double fl = y + x + r;
        double fr = y - x - r;
        double bl = y - x + r;
        double br = y + x - r;

        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr),
                        Math.max(Math.abs(bl), Math.abs(br)))));

        robot.lfMotor.setPower(fl / max);
        robot.rfMotor.setPower(fr / max);
        robot.lbMotor.setPower(bl / max);
        robot.rbMotor.setPower(br / max);
    }

}
