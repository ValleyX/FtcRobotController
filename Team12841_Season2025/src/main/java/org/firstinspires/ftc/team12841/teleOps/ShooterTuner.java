package org.firstinspires.ftc.team12841.teleOps;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.team12841.RobotHardware;


@TeleOp(name = "TUNE SHOOTER")
public class ShooterTuner extends LinearOpMode
{

    //public DcMotorEx shooterMotor;
    RobotHardware shooterHardware;
    RobotHardware robot;
    public double highVelocity = 2000;
    public double lowVelocity = 0;
    public double stop = 0;

    double curTargetVelocity = highVelocity;
    double F = 0;
    double P = 0;
    double[] stepSizes = {10.0, 1.0, 0.1,0.001,0.0001};
    int stepIndex = 1;

    private boolean lastBumper = false;
    private Timer shootTimer = new Timer();
    private boolean intaking = false;


    @Override
    public void runOpMode() throws InterruptedException {
        shooterHardware = new RobotHardware(this);
        robot = new RobotHardware(this);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        shooterHardware.shooterMotorRev.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooterHardware.shooterMotorBilda.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init Complete");

        waitForStart();


        while (opModeIsActive()) {
            //get all gamepad inputs
            if (gamepad1.yWasPressed()) {
                if (curTargetVelocity == highVelocity) {
                    curTargetVelocity = lowVelocity;
                } else {
                    curTargetVelocity = highVelocity;
                }
            }

            // allow to toggle through how big of step to make in P and F adjust
            if (gamepad1.bWasPressed()) {
                stepIndex = (stepIndex + 1) % stepSizes.length;
            }

            // Decrease F size
            if (gamepad1.dpadLeftWasPressed()){
                F -= stepSizes[stepIndex];
            }
            // Increase F size
            if (gamepad1.dpadRightWasPressed()) {
                F += stepSizes[stepIndex];
            }

            //Decress P size
            if (gamepad1.dpadDownWasPressed()){
                P -= stepSizes[stepIndex];
            }
            // Increase P size
            if (gamepad1.dpadUpWasPressed()) {
                P += stepSizes[stepIndex];
            }

            //Set new PIDF Coeff
            pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
            shooterHardware.shooterMotorRev.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            shooterHardware.shooterMotorBilda.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            //telemetry.addLine("Init Complete");

            //set velocity
            shooterHardware.setShooterRPM(curTargetVelocity);

            double curVelocity = (shooterHardware.shooterMotorBilda.getVelocity() * -60.0) / RobotHardware.ENCODER_TICS;
            double error = curTargetVelocity - curVelocity;

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
                robot.aimHood(0);
                robot.stopBallHold();
                if (!intaking) {
                    robot.stopFeed();
                }
            }

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

            lastBumper = gamepad1.right_bumper; // Track state for the next frame

            telemetry.addData("Target Velocity", curTargetVelocity);
            telemetry.addData("Current Velocity", "%.5f", curVelocity);
            telemetry.addData("Error", "%.2f", error);
            telemetry.addLine("--------------------------------");
            telemetry.addData("Tuning P", "%.4f (D-Pad U/D)",P);
            telemetry.addData("Tuning F", "%.4f (D-Pad L/R)",F);
            telemetry.addData("Step Size","%.4f (B Button)", stepSizes[stepIndex]);

            telemetry.update();

            /*
            to tune
            1) start with low velocity.
            2) Increase Feedforward (FF) until close to speed
            3) Switch to high velocity and see if speed over or undershoots.
            4) If overshoot lower FF, if undershoots increase FF.
            5) go to low velocity and see error, adjust until you get a good FF creates about the
            same error between high and low Velocity.
            6) Switch to the tenths place adjust (0.1) and try to get a value and try to get a FF
            value with low error.
            7) swap between high and low velocity and adjust.
            8) repeat until you can swap between high and low velocities with no or very low error.
            9) your low velocity may not be good.  that is OK, will fix with P.
            10) increase P, and swap between high and Low speed
            11) try to find the fastest time between high and low Velocity without overshooting.
            12) Want to get as stable as possible.
             */
        }

    }

}
