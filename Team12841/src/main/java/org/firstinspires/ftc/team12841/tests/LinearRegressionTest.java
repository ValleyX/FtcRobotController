package org.firstinspires.ftc.team12841.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team12841.RobotHardware;
import org.firstinspires.ftc.team12841.configs.TeleOpConfig;

import java.util.ArrayList;
import java.util.List;

/**
 * ShooterQuadRegressionTest
 *
 * Collects real-world shot data:
 *   • Limelight distance (d)
 *   • Shooter flywheel velocity (v = getVelocity())
 *   • Shooter motor power (p)
 *   • Turntable angle index (0,1,2)
 *
 * Fits a QUADRATIC model:
 *
 *      v = a*d² + b*d + c
 *
 * Coefficients update live in telemetry.
 *
 * The goal:
 *   Build a hyper-accurate model for autonomous + teleop shooting.
 *   You will take ~10–30 shots at different distances & TT angles.
 */
@Disabled
@TeleOp(name = "Shooter Quad Regression Test", group = "Tests")
public class LinearRegressionTest extends LinearOpMode {

    private RobotHardware robot;

    private final List<Double> distances  = new ArrayList<>();
    private final List<Double> velocities = new ArrayList<>();
    private final List<Double> powers     = new ArrayList<>();

    // Manual shooter power
    private double shooterPower = 0.70;

    // Turntable values
    private int ttIndex = 0;
    private final double[] TT_POS = {
            TeleOpConfig.TT_POS_0,
            TeleOpConfig.TT_POS_1,
            TeleOpConfig.TT_POS_2
    };

    // Buttons
    private boolean bPrev = false;
    private boolean rtPrev = false;
    private boolean rbPrev = false;
    private boolean lbPrev = false;
    private boolean aPrev  = false;
    private boolean dpadUpPrev = false;
    private boolean dpadDownPrev = false;

    // Shooter motor toggle
    private boolean shooterRunning = false;

    // Autocycle
    private boolean autoCycle = false;
    private double autoCycleStart = 0;

    // Regression coefficients
    private double coefA = 0;
    private double coefB = 0;
    private double coefC = 0;
    private boolean regressionReady = false;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotHardware(this);
        robot.innit(0);

        robot.turntableServo.setPosition(TT_POS[ttIndex]);
        robot.shooterServo.setPosition(TeleOpConfig.SHOOTER_IDLE);

        telemetry.addLine("Shooter Quad Regression Test — Ready");
        telemetry.addLine("Right Trigger = fire shot + log data");
        telemetry.addLine("Dpad Up/Down = adjust power");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ---------------- DRIVE (same as your teleOp) ----------------
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double r = -gamepad1.right_stick_x;

            double fl = y + x + r;
            double fr = y - x - r;
            double bl = y - x + r;
            double br = y + x - r;

            double max = Math.max(1.0,
                    Math.max(Math.abs(fl),
                            Math.max(Math.abs(fr),
                                    Math.max(Math.abs(bl), Math.abs(br)))));

            robot.lfMotor.setPower(fl / max);
            robot.rfMotor.setPower(fr / max);
            robot.lbMotor.setPower(bl / max);
            robot.rbMotor.setPower(br / max);

            // ---------------- SHOOTER MOTOR TOGGLE (B) ----------------
            boolean bNow = gamepad1.b;
            if (bNow && !bPrev) shooterRunning = !shooterRunning;
            bPrev = bNow;

            robot.shooterMotor.setPower(shooterRunning ? shooterPower : 0);

            // ---------------- POWER ADJUSTMENT (DPAD) ----------------
            if (gamepad1.dpad_up && !dpadUpPrev) shooterPower += 0.01;
            if (gamepad1.dpad_down && !dpadDownPrev) shooterPower -= 0.01;

            shooterPower = Math.max(0, Math.min(1, shooterPower));

            dpadUpPrev = gamepad1.dpad_up;
            dpadDownPrev = gamepad1.dpad_down;

            // ---------------- TURNTABLE ----------------
            if (gamepad1.right_bumper && !rbPrev) {
                ttIndex = (ttIndex + 1) % TT_POS.length;
                robot.turntableServo.setPosition(TT_POS[ttIndex]);
            }
            rbPrev = gamepad1.right_bumper;

            if (gamepad1.left_bumper && !lbPrev) {
                ttIndex--;
                if (ttIndex < 0) ttIndex = TT_POS.length - 1;
                robot.turntableServo.setPosition(TT_POS[ttIndex]);
            }
            lbPrev = gamepad1.left_bumper;

            // ---------------- AUTO-CYCLE (A) ----------------
            if (gamepad1.a && !aPrev && !autoCycle) {
                autoCycle = true;
                autoCycleStart = time;
            }
            aPrev = gamepad1.a;

            if (autoCycle) {
                int stage = (int)((time - autoCycleStart) / 1.5);
                if (stage < TT_POS.length)
                    robot.turntableServo.setPosition(TT_POS[stage]);
                else
                    autoCycle = false;
            }

            // ---------------- HEADING RESET ----------------
            if (gamepad1.guide) robot.resetPedroHeading();

            // ---------------- FIRE + LOG DATA (Right Trigger) ----------------
            boolean rtNow = gamepad1.right_trigger > 0.8;

            if (rtNow && !rtPrev) {

                // Spin-up
                robot.shooterMotor.setPower(shooterPower);
                sleep(2000);

                // Fire shot
                robot.shooterServo.setPosition(TeleOpConfig.SHOOTER_FIRE);
                sleep(250);
                robot.shooterServo.setPosition(TeleOpConfig.SHOOTER_IDLE);
                sleep(200);

                // Read sensors
                double d = robot.getBotDis();
                double v = robot.shooterMotor.getVelocity();

                if (d > 0 && v > 0) {
                    distances.add(d);
                    velocities.add(v);
                    powers.add(shooterPower);
                    computeRegression();
                }
            }
            rtPrev = rtNow;

            // ---------------- TELEMETRY ----------------
            telemetry.addLine("Shooter Regression:");
            telemetry.addData("Samples", distances.size());
            telemetry.addData("Power", "%.3f", shooterPower);
            telemetry.addData("TT Index", ttIndex);
            telemetry.addData("Distance", "%.1f", robot.getBotDis());
            telemetry.addData("Velocity", "%.0f", robot.shooterMotor.getVelocity());

            if (regressionReady) {
                telemetry.addLine("\nModel:");
                telemetry.addData("v = a*d² + b*d + c",
                        "\na=%.6f  b=%.6f  c=%.6f",
                        coefA, coefB, coefC);

                double dist = robot.getBotDis();
                double vel = coefA*dist*dist + coefB*dist + coefC;
                telemetry.addData("Predicted Vel @ %.1f in", "%.0f rpm", dist, vel);

                double k = estimateK();
                if (k > 0) {
                    double p = vel / k;
                    telemetry.addData("Predicted Power", "%.3f", p);
                }
            } else {
                telemetry.addLine("Need ≥ 3 samples for regression");
            }

            telemetry.update();
        }

        robot.shooterMotor.setPower(0);
    }

    // ============================================================
    // QUADRATIC REGRESSION (3x3 normal equation)
    // ============================================================
    private void computeRegression() {
        int n = distances.size();
        if (n < 3) {
            regressionReady = false;
            return;
        }

        double S0= n, S1=0,S2=0,S3=0,S4=0;
        double T0=0,T1=0,T2=0;

        for (int i=0;i<n;i++){
            double d = distances.get(i);
            double v = velocities.get(i);

            double d2 = d*d;
            double d3 = d2*d;
            double d4 = d3*d;

            S1+=d;
            S2+=d2;
            S3+=d3;
            S4+=d4;
            T0+=v;
            T1+=d*v;
            T2+=d2*v;
        }

        double det  = S0*(S2*S4 - S3*S3) - S1*(S1*S4 - S3*S2) + S2*(S1*S3 - S2*S2);
        double detC = T0*(S2*S4 - S3*S3) - S1*(T1*S4 - S3*T2) + S2*(T1*S3 - S2*T2);
        double detB = S0*(T1*S4 - S3*T2) - T0*(S1*S4 - S3*S2) + S2*(S1*T2 - T1*S2);
        double detA = S0*(S2*T2 - T1*S3) - S1*(S1*T2 - T1*S2) + T0*(S1*S3 - S2*S2);

        coefA = detA / det;
        coefB = detB / det;
        coefC = detC / det;

        regressionReady = true;
    }

    // Average velocity/power ratio
    private double estimateK() {
        double sum = 0;
        int count = 0;
        for (int i=0;i<velocities.size();i++) {
            double p = powers.get(i);
            if (p > 0) {
                sum += velocities.get(i) / p;
                count++;
            }
        }
        return count > 0 ? sum / count : -1;
    }
}
