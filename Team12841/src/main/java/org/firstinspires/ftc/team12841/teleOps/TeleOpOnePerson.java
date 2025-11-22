package org.firstinspires.ftc.team12841.teleOps;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.team12841.RobotHardware;
import org.firstinspires.ftc.team12841.configs.TeleOpConfig;

@TeleOp(name = "1 Person Drive", group = "opModes")
public class TeleOpOnePerson extends OpMode {

    // ---------- DECLARATIONS ----------
    private RobotHardware robot;

    // Pedro Vars
    private Follower follower;
    private boolean poseReady = false;

    // Shooter
    public boolean startShooter = false;
    private boolean bPrev = false;
    public double shootPwr = 1.0;
    private boolean DpadDownPrev = false;
    private boolean DpadUpPrev = false;

    // Baby Mode
    private boolean baby = false;
    private boolean babyPrev = false;

    // Ancient Mode
    private boolean oldMan = false;
    private boolean oldManPrev = false;

    // TT Vars
    private int ttIndex = 0;
    private boolean rbPrev = false, lbPrev = false;

    // Loading
    private boolean aPrev = false;
    private boolean autoCycle = false;
    private double autoCycleStart = 0;

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        robot = new RobotHardware(this);
        follower = robot.getFollower();

        robot.innit(0); // LL pipeline

        telemetry.addLine("Init OK — LL pipeline 0");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (follower != null) follower.update();

        if (follower != null && follower.getPose() != null) {
            poseReady = true;
            telemetry.addLine("POSE READY");
        } else {
            telemetry.addLine("Warming localization…");
        }
        telemetry.update();
    }

    @Override
    public void start() {
        if (follower != null) {
            follower.startTeleopDrive();
            follower.update();
        }
    }

    @Override
    public void loop() {

        if (follower == null) {
            driveManual();
            telemetry.addLine("FOLLOWER NULL — manual only");
            telemetry.update();
            return;
        }

        if (!poseReady) {
            follower.update();
            telemetry.addLine("Waiting for pose…");
            telemetry.update();
            return;
        }

        follower.update();

        if (follower.getPose() == null) {
            telemetry.addLine("POSE LOST");
            telemetry.update();
            return;
        }

        // ------------------ DRIVER INPUT ------------------
        double x = -gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double r = -gamepad1.right_stick_x;

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

        // Default drive (LL align only ADDS rotation)
        double rotCmd = getRotCmd(r);

        // Final Pedro drive
        follower.setTeleOpDrive(y, x, rotCmd, false);

        // ------------------ AUTO BRAKE WHEN STICKS RELEASED ------------------
        if (Math.abs(x) < 0.02 && Math.abs(y) < 0.02 && Math.abs(r) < 0.02 &&
                gamepad1.left_trigger < 0.8) {
            robot.applyFullBrake();
        }

        // ------------------ BABY MODE ------------------
        if (gamepad1.back && !babyPrev) baby = !baby;
        babyPrev = gamepad1.back;

        // ------------------ ANCIENT MODE ------------------
        if (gamepad1.start && !oldManPrev) oldMan = !oldMan;
        oldManPrev = gamepad1.start;

        // ------------------ SHOOTER SERVO ------------------
        if (gamepad1.right_trigger >= 0.8) {
            robot.shooterServo.setPosition(TeleOpConfig.SHOOTER_FIRE);
        } else {
            robot.shooterServo.setPosition(TeleOpConfig.SHOOTER_IDLE);
        }

        // ------------------ SHOOTER POWER DPAD ------------------
        if (gamepad1.dpad_down && !DpadDownPrev) {
            shootPwr -= 0.025;
            if (shootPwr < 0) shootPwr = 0;
        }
        DpadDownPrev = gamepad1.dpad_down;

        if (gamepad1.dpad_up && !DpadUpPrev) {
            shootPwr += 0.025;
            if (shootPwr > 1) shootPwr = 1;
        }
        DpadUpPrev = gamepad1.dpad_up;

        // ------------------ SHOOTER POWER PRESETS ------------------
        if (gamepad1.dpad_right) {
            shootPwr = 0.675;
        }

        if (gamepad1.dpad_left) {
            shootPwr = 0.75;
        }

        // ------------------ SHOOTER MOTOR TOGGLE ------------------
        if (gamepad1.b && !bPrev) startShooter = !startShooter;
        bPrev = gamepad1.b;

        robot.shooterMotor.setPower(startShooter ? shootPwr : 0);

        // ------------------ TURNTABLE ------------------
        double[] tt = {TeleOpConfig.TT_POS_0, TeleOpConfig.TT_POS_1, TeleOpConfig.TT_POS_2};

        if (gamepad1.right_bumper && !rbPrev &&
                robot.shooterServo.getPosition() == TeleOpConfig.SHOOTER_IDLE) {
            ttIndex = (ttIndex + 1) % tt.length;
            robot.turntableServo.setPosition(tt[ttIndex]);
        }
        rbPrev = gamepad1.right_bumper;

        if (gamepad1.left_bumper && !lbPrev &&
                robot.shooterServo.getPosition() == TeleOpConfig.SHOOTER_IDLE) {
            ttIndex--;
            if (ttIndex < 0) ttIndex = tt.length - 1;
            robot.turntableServo.setPosition(tt[ttIndex]);
        }
        lbPrev = gamepad2.left_bumper;

        // ------------------ LOAD CYCLE ------------------
        if (gamepad1.a && !aPrev && !autoCycle) {
            autoCycle = true;
            autoCycleStart = runtime.seconds();
        }
        aPrev = gamepad1.a;

        if (autoCycle) {
            int stage = (int)((runtime.seconds() - autoCycleStart) / 1.5);
            if (stage < tt.length) {
                ttIndex = stage;
                robot.turntableServo.setPosition(tt[stage]);
            } else {
                autoCycle = false;
            }
        }

        // Reset IMU
        if (gamepad1.guide)
            robot.resetPedroHeading();

        // ------------------ TELEMETRY ------------------
        telemetry.addData("Heading:      ", follower.getPose().getHeading());
        telemetry.addData("LL Aligning:  ", gamepad1.left_trigger > 0.8);
        telemetry.addData("Shoot Power:  ", shootPwr);
        telemetry.addData("Shooter Motor:", startShooter);
        telemetry.addData("Baby Mode:    ", baby);
        telemetry.addData("Ancient Mode: ", oldMan);
        telemetry.addData("Turntable Pos:", ttIndex);
        telemetry.update();
    }

    private double getRotCmd(double r) {
        double rotCmd = r;

        // ------------------ A-MODE LL AIM (ONLY ROTATES) ------------------
        if (gamepad1.left_trigger > 0.8) {

            double tx = robot.getTx(); // NEGATIVE stays negative
            if (tx != -999) {

                double Kp = TeleOpConfig.KP;
                double turn = -(tx * Kp);  // ← KEEP NEGATIVE

                double maxTurn = 0.16;
                if (turn > maxTurn) turn = maxTurn;
                if (turn < -maxTurn) turn = -maxTurn;

                // ROTATION blended with driver turn input
                rotCmd += turn;
            }
        }
        return rotCmd;
    }

    // ------------------ MANUAL FALLBACK ------------------
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
