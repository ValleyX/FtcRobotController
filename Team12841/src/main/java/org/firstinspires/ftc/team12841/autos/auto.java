package org.firstinspires.ftc.team12841.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.team12841.RobotHardware;

@Autonomous(name = "Auto Template", group = "Autonomous")
public class auto extends LinearOpMode {

    private RobotHardware robot;

    // === CONSTANTS ===
    private static final double TICKS_PER_REV = 383.6; // GoBilda 435 RPM Yellow Jacket
    private static final double WHEEL_DIAMETER_INCHES = 3.78; // GoBilda Mecanum or similar
    private static final double GEAR_RATIO = 1.0; // external gearing, 1:1 unless modified
    private static final double TICKS_PER_INCH =
            (TICKS_PER_REV * GEAR_RATIO) / (Math.PI * WHEEL_DIAMETER_INCHES);

    // Speeds
    private static final double DRIVE_SPEED = 0.5;
    private static final double STRAFE_SPEED = 0.5;
    private static final double TURN_SPEED = 0.4;

    @Override
    public void runOpMode() {
        robot = new RobotHardware(this);

        telemetry.addLine("Status: Initialized");
        telemetry.addData("Ticks per Inch", "%.2f", TICKS_PER_INCH);
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // DURING DRIVING
        }
    }

    // === MOVEMENT METHODS ===

    public void driveForward(double inches) {
        encoderDrive(DRIVE_SPEED, inches, inches, inches, inches);
    }

    public void driveBackward(double inches) {
        encoderDrive(DRIVE_SPEED, -inches, -inches, -inches, -inches);
    }

    public void strafeRight(double inches) {
        encoderDrive(STRAFE_SPEED, inches, -inches, -inches, inches);
    }

    public void strafeLeft(double inches) {
        encoderDrive(STRAFE_SPEED, -inches, inches, inches, -inches);
    }

    public void turnRight(double degrees) {
        double turnInches = degreesToInches(degrees);
        encoderDrive(TURN_SPEED, turnInches, -turnInches, turnInches, -turnInches);
    }

    public void turnLeft(double degrees) {
        double turnInches = degreesToInches(degrees);
        encoderDrive(TURN_SPEED, -turnInches, turnInches, -turnInches, turnInches);
    }

    // === CORE ENCODER LOGIC ===
    private void encoderDrive(double speed,
                              double lfInches, double rfInches,
                              double lbInches, double rbInches) {

        int newLfTarget = robot.lfMotor.getCurrentPosition() + (int) (lfInches * TICKS_PER_INCH);
        int newRfTarget = robot.rfMotor.getCurrentPosition() + (int) (rfInches * TICKS_PER_INCH);
        int newLbTarget = robot.lbMotor.getCurrentPosition() + (int) (lbInches * TICKS_PER_INCH);
        int newRbTarget = robot.rbMotor.getCurrentPosition() + (int) (rbInches * TICKS_PER_INCH);

        // Set target positions
        robot.lfMotor.setTargetPosition(newLfTarget);
        robot.rfMotor.setTargetPosition(newRfTarget);
        robot.lbMotor.setTargetPosition(newLbTarget);
        robot.rbMotor.setTargetPosition(newRbTarget);

        // Enable RUN_TO_POSITION
        robot.lfMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rfMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lbMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rbMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Apply power
        robot.lfMotor.setPower(Math.abs(speed));
        robot.rfMotor.setPower(Math.abs(speed));
        robot.lbMotor.setPower(Math.abs(speed));
        robot.rbMotor.setPower(Math.abs(speed));

        // Monitor until complete
        while (opModeIsActive() &&
                (robot.lfMotor.isBusy() || robot.rfMotor.isBusy() ||
                        robot.lbMotor.isBusy() || robot.rbMotor.isBusy())) {

            telemetry.addData("LF", "%d / %d", robot.lfMotor.getCurrentPosition(), newLfTarget);
            telemetry.addData("RF", "%d / %d", robot.rfMotor.getCurrentPosition(), newRfTarget);
            telemetry.addData("LB", "%d / %d", robot.lbMotor.getCurrentPosition(), newLbTarget);
            telemetry.addData("RB", "%d / %d", robot.rbMotor.getCurrentPosition(), newRbTarget);
            telemetry.update();
        }

        stopAllMotors();

        // Reset encoders to RUN_USING_ENCODER
        robot.lfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void stopAllMotors() {
        robot.lfMotor.setPower(0);
        robot.rfMotor.setPower(0);
        robot.lbMotor.setPower(0);
        robot.rbMotor.setPower(0);
    }

    private double degreesToInches(double degrees) {
        // Rotations for auto
        double ROBOT_DIAMETER_INCHES = 18.0; // ADJUST THIS
        double ROBOT_CIRCUMFERENCE = Math.PI * ROBOT_DIAMETER_INCHES;
        return ROBOT_CIRCUMFERENCE * (degrees / 360.0);
    }
}
