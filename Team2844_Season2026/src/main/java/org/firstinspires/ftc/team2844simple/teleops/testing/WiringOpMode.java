package org.firstinspires.ftc.team2844simple.teleops.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.vcs.valleylib.ftc.opmode.CommandOpMode;

import org.firstinspires.ftc.team2844simple.helpers.Constants;

@TeleOp(name = "Simple Wiring OpMode")
public class WiringOpMode extends CommandOpMode {
    DcMotor leftFront, leftBack, rightFront, rightBack;
    DcMotor pollen;
    DcMotor intake, transfer;
    Servo gate1, gate2;

    int testStep;
    boolean nextStep;
    boolean b;


    @Override
    protected void initialize() {
        /* --- Drive Motors --- */
        leftFront = hardwareMap.get(DcMotor.class, Constants.CHM0);
        leftBack = hardwareMap.get(DcMotor.class, Constants.CHM1);
        rightBack = hardwareMap.get(DcMotor.class, Constants.CHM2);
        rightFront = hardwareMap.get(DcMotor.class, Constants.CHM3);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        /* --- Intake/Transfer Motors --- */
        intake = hardwareMap.get(DcMotor.class, Constants.EHM0);
        transfer = hardwareMap.get(DcMotor.class, Constants.EHM1);

        /* --- Shooter Motor --- */
        pollen = hardwareMap.get(DcMotor.class, Constants.EHM3);

        /* --- Servos --- */
        gate1 = hardwareMap.get(Servo.class, Constants.CHS0);
        gate2 = hardwareMap.get(Servo.class, Constants.EHS0);
        gate1.setDirection(Servo.Direction.REVERSE);
        gate2.setDirection(Servo.Direction.FORWARD);

        /* --- Variables --- */
        testStep = 0;
        nextStep = false;
        b = false;

    }

    @Override
    protected void run() {
        //? change the step to the next or previous testing
        if(gamepad1.rightBumperWasPressed()){
            testStep++;
        } else if(gamepad1.leftBumperWasPressed()){
            testStep--;
        }

        //? Loops the steps
        while (testStep < 0) testStep+=3;
        while (testStep > 2) testStep-=3;

        //? Changes what the motors test
        switch (testStep){
            case (0):
                testDrive();
                driveTelemetry();
                break;
            case (1):
                testMotors();
                motorTelemetry();
                break;
            case (2):
                testServos();
                servoTelemetry();
                break;
            default:
                break;
        }
    }

    /* ----- TEST DRIVES ----- */
    public void testDrive(){
        if (gamepad1.x){
            leftFront.setPower(1.0);
        } else if(gamepad1.a){
            rightFront.setPower(1.0);
        } else if(gamepad1.b){
            rightBack.setPower(1.0);
        } else if(gamepad1.y){
            leftBack.setPower(1.0);
        } else {
            leftFront.setPower(0.0);
            leftBack.setPower(0.0);
            rightFront.setPower(0.0);
            rightBack.setPower(0.0);
        }
    }

    public void driveTelemetry(){
        telemetryBus.put("Testing Drive", "");
        telemetryBus.put("Left Front Motor: ", leftFront.getPower());
        telemetryBus.put("Left Back Motor: ", leftBack.getPower());
        telemetryBus.put("Right Front Motor: ", rightFront.getPower());
        telemetryBus.put("Right Back Motor: ", rightBack.getPower());
    }

    /* ----- TEST MOTORS ----- */
    public void testMotors(){
        if (gamepad1.x){
            intake.setPower(Constants.INTAKE_SPEED);
        } else if(gamepad1.a){
            transfer.setPower(Constants.TRANSFER_SPEED);
        } else if(gamepad1.y){
            pollen.setPower(1.0);
        } else {
            intake.setPower(0.0);
            transfer.setPower(0.0);
            pollen.setPower(0.0);
        }
    }

    public void motorTelemetry(){
        telemetryBus.put("Testing Motors", "");
        telemetryBus.put("Intake Motor: ", intake.getPower());
        telemetryBus.put("Transfer Motor: ", transfer.getPower());
        telemetryBus.put("Pollen Motor: ", pollen.getPower());
    }


    /* ----- SERVOS ----- */
    public void testServos(){
        if(gamepad1.bWasPressed()){
            b = !b;
            double pos = (b)? Constants.OPEN_GATE: Constants.CLOSED_GATE;
            gate1.setPosition(pos);
            gate2.setPosition(pos);
        }
    }

    public void servoTelemetry(){
        telemetryBus.put("Testing Servos", "");
        telemetryBus.put("Gate Poses: ", gate1.getPosition());
    }
}
