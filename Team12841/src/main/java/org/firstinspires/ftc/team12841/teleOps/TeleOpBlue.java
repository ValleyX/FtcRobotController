package org.firstinspires.ftc.team12841.opmodes.teleop;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team12841.RobotHardware;
import org.firstinspires.ftc.team12841.configs.PanelsConfig;

@TeleOp(name = "Blue 2 People", group = "TeleOp")
public class TeleOpBlue extends OpMode {

    private RobotHardware robot;
    private Follower follower;

    /* ===================== STATE ===================== */

    @Override
    public void init() {
        robot = new RobotHardware(this);
        follower = robot.getFollower();

        if (robot.limelight != null) {
            robot.limelight.pipelineSwitch(1);
        }

        telemetry.addLine("Pedro Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        /* ===================== INPUT ===================== */

        double forward = -gamepad1.left_stick_y;
        double strafe  =  gamepad1.left_stick_x;
        double turn    =  gamepad1.right_stick_x;

        boolean llAlignEnabled = gamepad1.right_bumper;

        robot.updateLLHeading();

        /* ===================== DRIVE ===================== */

        if (llAlignEnabled) {
            limelightAlignDrive(forward, strafe);
        } else {
            manualDrive(forward, strafe, turn);
        }

        /* ===================== MECHANISMS ===================== */

        handleIntake();
        handleShooter();

        /* ===================== TELEMETRY ===================== */

        telemetry.addData("LL Align", llAlignEnabled);
        telemetry.addData("tx", robot.getTx());
        telemetry.addData("Heading", robot.headingDeg());
        telemetry.update();
    }

    /* ===================== DRIVE METHODS ===================== */

    private void manualDrive(double forward, double strafe, double turn) {
        follower.setTeleOpDrive(
                forward,
                strafe,
                turn
        );
        follower.update();
    }

    private void limelightAlignDrive(double forward, double strafe) {
        double tx = robot.getTx();

        if (tx == -999) {
            manualDrive(forward, strafe, 0);
            return;
        }

        double turnCorrection = tx * PanelsConfig.LLPGAIN;

        follower.setTeleOpDrive(
                forward,
                strafe,
                -turnCorrection
        );
        follower.update();
    }

    /* ===================== INTAKE PLACEHOLDERS ===================== */

    private void handleIntake() {
        if (gamepad2.right_trigger > 0.5) {
            intakeOn();
        } else if (gamepad2.left_trigger > 0.5) {
            intakeReverse();
        } else {
            intakeOff();
        }
    }

    private void intakeOn() {
        // TODO: robot.runIntake(+1.0);
    }

    private void intakeReverse() {
        // TODO: robot.runIntake(-1.0);
    }

    private void intakeOff() {
        // TODO: robot.stopIntake();
    }

    /* ===================== SHOOTER PLACEHOLDERS ===================== */

    private void handleShooter() {
        if (gamepad2.right_bumper) {
            shooterOn();
        } else if (gamepad2.left_bumper) {
            shooterOff();
        }
    }

    private void shooterOn() {
        // TODO: robot.setShooterRPM(PanelsConfig.SHOOTER_RPM);
    }

    private void shooterOff() {
        // TODO: robot.stopShooter();
    }
}
