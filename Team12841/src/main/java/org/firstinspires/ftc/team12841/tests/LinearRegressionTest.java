package org.firstinspires.ftc.team12841.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team12841.RobotHardware;
import org.firstinspires.ftc.team12841.configs.TeleOpConfig;

import java.util.ArrayList;
import java.util.List;

/**
 * ShooterQuadRegressionTest
 *
 * Collects data samples:
 *   - Limelight distance
 *   - Shooter wheel velocity (RPM)
 *   - Shooter motor power
 *   - Turntable position
 * Fires a shot and logs a sample on button press.
 * Then computes a **quadratic regression** live:
 *
 *   velocity = a*distance^2 + b*distance + c
 *
 * Displays the fitted coefficients (a, b, c) in telemetry.
 * Once regression is meaningful, it will also show the predicted power
 * needed for a target distance.
 */
@TeleOp(name = "Shooter Quad Regression Test", group = "Tests")
public class LinearRegressionTest extends LinearOpMode {

    private RobotHardware robot;

    private final List<Double> distances = new ArrayList<>();
    private final List<Double> velocities = new ArrayList<>();
    private final List<Double> powers     = new ArrayList<>();

    // Shooter power manual start
    private double shooterPower = 0.70;

    // Turntable states
    private int ttIndex = 0;
    private final double[] TT_POSITIONS = {
            TeleOpConfig.TT_POS_0,
            TeleOpConfig.TT_POS_1,
            TeleOpConfig.TT_POS_2
    };

    // Button state trackers
    private boolean xPrev = false;
    private boolean dpadUpPrev = false;
    private boolean dpadDownPrev = false;

    // Regression coefficients
    private double coefA = 0.0;
    private double coefB = 0.0;
    private double coefC = 0.0;
    private boolean regressionReady = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(this);
        robot.innit(0);  // Limelight pipeline

        // Setup initial servo positions
        robot.turntableServo.setPosition(TT_POSITIONS[ttIndex]);
        robot.shooterServo.setPosition(TeleOpConfig.SHOOTER_IDLE);
        robot.shooterMotor.setPower(0);

        telemetry.addLine("Shooter Quad Regression Test READY");
        telemetry.addLine("Use D-Pad to adjust power, X to fire and log");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Adjust shooter power
            if (gamepad1.dpad_up && !dpadUpPrev) {
                shooterPower += 0.01;
            }
            if (gamepad1.dpad_down && !dpadDownPrev) {
                shooterPower -= 0.01;
            }
            dpadUpPrev   = gamepad1.dpad_up;
            dpadDownPrev = gamepad1.dpad_down;

            // Clamp power
            shooterPower = Math.max(0.0, Math.min(1.0, shooterPower));

            // Fire + log on X press
            boolean xNow = gamepad1.x;
            if (xNow && !xPrev) {
                // Start shooter
                robot.shooterMotor.setPower(shooterPower);
                sleep(1000); // warm-up

                // Fire servo
                robot.shooterServo.setPosition(TeleOpConfig.SHOOTER_FIRE);
                sleep(250);
                robot.shooterServo.setPosition(TeleOpConfig.SHOOTER_IDLE);

                // settle
                sleep(200);

                // Read data
                double d  = robot.getBotDis();
                double v  = robot.shooterMotor.getVelocity();
                double p  = shooterPower;

                if (d > 0 && v > 0) {
                    distances.add(d);
                    velocities.add(v);
                    powers.add(p);

                    // Compute regression
                    computeQuadraticRegression();
                }

                // Step turret
                ttIndex = (ttIndex + 1) % TT_POSITIONS.length;
                robot.turntableServo.setPosition(TT_POSITIONS[ttIndex]);
            }
            xPrev = xNow;

            // Telemetry
            telemetry.addLine("Shooter Quad Regression Test");
            telemetry.addData("Samples", distances.size());
            telemetry.addData("Current Power", "%.3f", shooterPower);
            telemetry.addData("TT Index", ttIndex);
            telemetry.addData("TT Pos", "%.3f", TT_POSITIONS[ttIndex]);
            telemetry.addData("LL Distance", "%.2f", robot.getBotDis());

            if (regressionReady) {
                telemetry.addLine("");
                telemetry.addLine("Fitted model:");
                telemetry.addData("velocity = %.6f*d^2 + %.6f*d + %.6f", String.valueOf(coefA), coefB, coefC);

                // Example: predict velocity, then convert to power
                double targetDist = robot.getBotDis();
                double predictedVel = coefA * targetDist * targetDist
                        + coefB * targetDist
                        + coefC;
                telemetry.addData("Predicted velocity @ %.2f in = %.0f rpm", String.valueOf(targetDist), predictedVel);

                // Estimate k factor (average vel/power)
                double k = estimateK();
                if (k > 0) {
                    double predictedPower = predictedVel / k;
                    telemetry.addData("Estimated k (vel/power) = %.3f", k);
                    telemetry.addData("Predicted power = %.3f", predictedPower);
                }
            } else {
                telemetry.addLine("");
                telemetry.addLine("Need at least 3 valid samples for quadratic regression.");
            }

            telemetry.update();
        }

        // Stop shooter
        robot.shooterMotor.setPower(0);
    }

    /**
     * Computes coefficients a, b, c for
     *   velocity = a*d^2 + b*d + c
     * using least-squares method.
     */
    private void computeQuadraticRegression() {
        int n = distances.size();
        if (n < 3) {
            regressionReady = false;
            return;
        }

        // Sums
        double sumD   = 0;
        double sumD2  = 0;
        double sumD3  = 0;
        double sumD4  = 0;
        double sumV   = 0;
        double sumDv  = 0;
        double sumD2v = 0;

        for (int i = 0; i < n; i++) {
            double d  = distances.get(i);
            double v  = velocities.get(i);

            double d2 = d * d;
            double d3 = d2 * d;
            double d4 = d3 * d;

            sumD   += d;
            sumD2  += d2;
            sumD3  += d3;
            sumD4  += d4;
            sumV   += v;
            sumDv  += d * v;
            sumD2v += d2 * v;
        }

        // Set up normal equations:
        // [ n     Σd     Σd^2 ] [ c ]   [ Σv     ]
        // [ Σd   Σd^2   Σd^3 ] [ b ] = [ Σdv    ]
        // [ Σd^2 Σd^3   Σd^4 ] [ a ]   [ Σd2v   ]

        double S0  = n;
        double S1  = sumD;
        double S2  = sumD2;
        double S3  = sumD3;
        double S4  = sumD4;
        double T0  = sumV;
        double T1  = sumDv;
        double T2  = sumD2v;

        // Solve using Cramer’s rule (3x3)
        double det  =   S0*(S2*S4 - S3*S3)
                - S1*(S1*S4 - S3*S2)
                + S2*(S1*S3 - S2*S2);

        double detA =   T0*(S2*S4 - S3*S3)
                - S1*(T1*S4 - S3*T2)
                + S2*(T1*S3 - S2*T2);

        double detB =   S0*(T1*S4 - S3*T2)
                - T0*(S1*S4 - S3*S2)
                + S2*(S1*T2 - T1*S2);

        double detC =   S0*(S2*T2 - T1*S3)
                - S1*(S1*T2 - T1*S2)
                + T0*(S1*S3 - S2*S2);

        coefA = detA / det;
        coefB = detB / det;
        coefC = detC / det;

        regressionReady = true;
    }

    /**
     * Estimate constant k = average(vel / power)
     * using all logged samples.
     */
    private double estimateK() {
        double sum = 0;
        int count = 0;
        for (int i = 0; i < velocities.size(); i++) {
            double v = velocities.get(i);
            double p = powers.get(i);
            if (p > 0) {
                sum += v / p;
                count++;
            }
        }
        return count > 0 ? sum / count : -1;
    }
}
