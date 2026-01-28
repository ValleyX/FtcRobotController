package org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Hardwares.ShooterHardware;

@TeleOp(name = "Flywheel tune")
@Disabled
public class Flywheel_Tune extends LinearOpMode
{

    //public DcMotorEx shooterMotor;
    ShooterHardware shooterHardware;

    public double highVelocity = 4500;
    public double lowVelocity = 3000;

    double curTargetVelocity = highVelocity;
    double F = 0;
    double P = 0;
    double[] stepSizes = {10.0, 1.0, 0.1,0.001,0.0001};
    int stepIndex = 1;


    @Override
    public void runOpMode() throws InterruptedException {
        shooterHardware = new ShooterHardware(this);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        shooterHardware.shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init Complete");

        waitForStart();


        while (opModeIsActive()) {
            //get all gamepad inputs
            if (gamepad1.yWasPressed()) {
                if (curTargetVelocity == highVelocity) {
                    curTargetVelocity = lowVelocity;
                } else { curTargetVelocity = highVelocity; }
            }

            //allow to toggle through how big of step to make in P and F adjust
            if (gamepad1.bWasPressed()) {
                stepIndex = (stepIndex + 1) % stepSizes.length;
            }

            //Decress F size
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
            shooterHardware.shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            //telemetry.addLine("Init Complete");

            //set velocity
            shooterHardware.shooterMotor.setVelocity(curTargetVelocity);

            double curVelocity = shooterHardware.shooterMotor.getVelocity();
            double error = curTargetVelocity - curVelocity;

            telemetry.addData("Target Velocity", curTargetVelocity);
            telemetry.addData("Current Velocity", "%.5f", curVelocity);
            telemetry.addData("Error", "%.2f", error);
            telemetry.addLine("--------------------------------");
            telemetry.addData("Tuning P", "%.4f (D-Pad U/D)",P);
            telemetry.addData("Tuning F", "%.4f (D-Pad L/R)",F);
            telemetry.addData("Step Size","%.54f (B Button)", stepSizes[stepIndex]);

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

