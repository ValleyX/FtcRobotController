package org.firstinspires.ftc.team12841.teleOps;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team12841.RobotHardware;
import org.firstinspires.ftc.team12841.configs.TeleOpConfig;

@TeleOp(name = "TeleOp Main", group = "opModes")
public class teleOpPathing extends OpMode {

    private RobotHardware robot;
    private Follower follower;

    private boolean poseReady = false;

    private boolean shooterActive = false;
    private boolean bPrev = false;
    private double shooterStart = 0;

    private int ttIndex = 0;
    private boolean lbPrev = false, rbPrev = false;
    private boolean aPrev = false;
    private boolean autoCycle = false;
    private double autoCycleStart = 0;

    private boolean baby = false;
    private boolean babyPrev = false;

    private boolean startShooter = false;

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        robot = new RobotHardware(this);
        follower = robot.getFollower();

        telemetry.addLine("Init OK");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (follower != null) follower.update();
        if (follower != null && follower.getPose() != null) {
            poseReady = true;
            telemetry.addLine("POSE READY");
        } else {
            telemetry.addLine("Warming...");
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
    //Shooter Power
    double shootPwr = 1;  //set to 100% for default
    boolean Dpad_Status = false;
    boolean Dpad_Status_up = false;

    @Override
    public void loop() {
        if (follower == null) {
            telemetry.addLine("FOLLOWER NULL — manual drive");
            driveManual();
            telemetry.update();
            return;
        }
        if (!poseReady) {
            follower.update();
            telemetry.addLine("Waiting for pose...");
            telemetry.update();
            return;
        }

        follower.update();
        if (follower.getPose() == null) {
            telemetry.addLine("POSE NULL");
            telemetry.update();
            return;
        }

        // Drive inputs
        double x = -gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double r = -gamepad1.right_stick_x;

        if (baby) {
            x *= TeleOpConfig.BABY_MODE_SCALE;
            y *= TeleOpConfig.BABY_MODE_SCALE;
            r *= TeleOpConfig.BABY_MODE_SCALE;
        }



        follower.setTeleOpDrive(y, x, r, false);

        // Baby mode toggle
        if (gamepad1.right_bumper && !babyPrev) {
            baby = !baby;
        }
        babyPrev = gamepad1.right_bumper;

        // Shooter trigger
        if (gamepad2.right_trigger >= 0.8) {
            robot.shooterServo.setPosition(TeleOpConfig.SHOOTER_FIRE);
        } else {
            robot.shooterServo.setPosition(TeleOpConfig.SHOOTER_IDLE);
        }
//******************************JAE CODE****************************************
        if (gamepad2.dpad_down == true && shootPwr >0) {
            if(!Dpad_Status){
                shootPwr = shootPwr-0.025;
                Dpad_Status = true;
            }
        } else {
            Dpad_Status = false;
        }
        if (gamepad2.dpad_up == true && shootPwr < 1) {
            if(!Dpad_Status_up){
                shootPwr = shootPwr+0.025;
                Dpad_Status_up = true;
            }
        } else {
            Dpad_Status_up = false;
        }
        telemetry.addData("Shoot Pwr = ", shootPwr);

//        if (gamepad2.b && !bPrev) {
//            //shooterActive = !shooterActive;
//            robot.shooterMotor.setPower(shootPwr);
//            bPrev = true;
//        }
//        if (gamepad2.b && bPrev)  {
//            robot.shooterMotor.setPower(0.0);
//            bPrev = false;
//        }
        if(gamepad2.b){
            if(!bPrev){
                startShooter = !startShooter;
                bPrev = true;
            }
        } else {
            bPrev = false;
        }

        if(startShooter){
            robot.shooterMotor.setPower(shootPwr);
        } else {
            robot.shooterMotor.setPower(0.0);
        }

        //*******************************END JAE CODE *****************************
        // Shooter toggle with auto-off after 10s
       /* if (gamepad2.b && !bPrev) {
            shooterActive = !shooterActive;
            shooterStart = runtime.seconds();
        }
        bPrev = gamepad2.b;

        if (shooterActive) {
            robot.shooterMotor.setPower(1.0);
            bPrev = true;
            if (runtime.seconds() - shooterStart >= 7) {
                shooterActive = false;
                robot.shooterMotor.setPower(0.0);
            }
        }
*/
        // Turntable bumpers
        double[] ttPositions = {
                TeleOpConfig.TT_POS_0,
                TeleOpConfig.TT_POS_1,
                TeleOpConfig.TT_POS_2
        };

        if (gamepad2.right_bumper && !rbPrev && (robot.shooterServo.getPosition() == TeleOpConfig.SHOOTER_IDLE)) {
            ttIndex = (ttIndex + 1) % ttPositions.length;
            robot.turntableServo.setPosition(ttPositions[ttIndex]);
        }
        if (gamepad2.left_bumper && !lbPrev && (robot.shooterServo.getPosition() == TeleOpConfig.SHOOTER_IDLE)) {
            ttIndex--;
            if (ttIndex < 0) {
                ttIndex = ttPositions.length - 1;
            }
            robot.turntableServo.setPosition(ttPositions[ttIndex]);
        }
        rbPrev = gamepad2.right_bumper;
        lbPrev = gamepad2.left_bumper;

        // Auto cycle positions with A
        if (gamepad2.a && !aPrev && !autoCycle) {
            autoCycle = true;
            autoCycleStart = runtime.seconds();
        }
        aPrev = gamepad2.a;

        if (autoCycle) {
            int stage = (int)((runtime.seconds() - autoCycleStart) / 1.5);
            if (stage < ttPositions.length) {
                ttIndex = stage;
                robot.turntableServo.setPosition(ttPositions[stage]);
            } else {
                autoCycle = false;
            }
        }

        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Baby Mode", baby);
        telemetry.addData("Shooter Active", shooterActive);
        telemetry.update();
    }

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
