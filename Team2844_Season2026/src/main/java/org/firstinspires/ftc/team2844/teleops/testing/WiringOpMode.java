package org.firstinspires.ftc.team2844.teleops.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.vcs.valleylib.ftc.opmode.CommandOpMode;

import org.firstinspires.ftc.team2844.helpers.Constants;

@TeleOp(name = "Wiring OpMode")
public class WiringOpMode extends CommandOpMode {
    DcMotor leftFront, leftBack, rightFront, rightBack;
    DcMotor nectar, pollen;
    DcMotor intake, transfer;
    Servo axon1, axon2, hood1, hood2, nectarGate, pollenGate, flower;
    AnalogInput encoder1, encoder2;
    Servo light0, light1, light2;
    DigitalChannel n0, n1, n2, n3, p0, p1, p2, p3;

    int testStep;
    boolean nextStep;
    boolean x,a,b,y;


    @Override
    protected void initialize() {
        /* --- Drive Motors --- */
        leftFront = hardwareMap.get(DcMotor.class, Constants.CHM0);
        leftBack = hardwareMap.get(DcMotor.class, Constants.CHM1);
        rightFront = hardwareMap.get(DcMotor.class, Constants.CHM2);
        rightBack = hardwareMap.get(DcMotor.class, Constants.CHM3);

        /* --- Intake/Transfer Motors --- */
        intake = hardwareMap.get(DcMotor.class, Constants.EHM0);
        transfer = hardwareMap.get(DcMotor.class, Constants.EHM1);

        /* --- Shooter Motors --- */
        nectar = hardwareMap.get(DcMotor.class, Constants.EHM2);
        pollen = hardwareMap.get(DcMotor.class, Constants.EHM3);

        /* --- Servos --- */
        axon1 = hardwareMap.get(Servo.class, Constants.CHS4);
        axon2 = hardwareMap.get(Servo.class, Constants.CHS5);

        hood1 = hardwareMap.get(Servo.class, Constants.CHS0);
        hood2 = hardwareMap.get(Servo.class, Constants.CHS1);

        nectarGate = hardwareMap.get(Servo.class, Constants.CHS2);
        pollenGate = hardwareMap.get(Servo.class, Constants.CHS3);
        flower = hardwareMap.get(Servo.class, Constants.EHS0);

        /* --- Turret Encoders --- */
        encoder1 = hardwareMap.get(AnalogInput.class, Constants.CHDD0);
        encoder2 = hardwareMap.get(AnalogInput.class, Constants.CHDD1);


        /* --- Lights --- */
        light0 = hardwareMap.get(Servo.class, Constants.EHS3);
        light1 = hardwareMap.get(Servo.class, Constants.EHS4);
        light2 = hardwareMap.get(Servo.class, Constants.EHS5);

        /* --- Beam Breaks --- */
        n0 = hardwareMap.get(DigitalChannel.class, Constants.EHDD0);
        n1 = hardwareMap.get(DigitalChannel.class, Constants.EHDD1);
        n2 = hardwareMap.get(DigitalChannel.class, Constants.EHDD2);
        n3 = hardwareMap.get(DigitalChannel.class, Constants.EHDD3);

        p0 = hardwareMap.get(DigitalChannel.class, Constants.EHDD4);
        p1 = hardwareMap.get(DigitalChannel.class, Constants.EHDD5);
        p2 = hardwareMap.get(DigitalChannel.class, Constants.EHDD6);
        p3 = hardwareMap.get(DigitalChannel.class, Constants.EHDD7);

        /* --- Variables --- */
        testStep = 0;
        nextStep = false;
        x = false;
        a = false;
        b = false;
        y = false;

    }

    @Override
    protected void run() {
        //? Lights
        light0.setPosition(Constants.GREEN);
        light1.setPosition(Constants.YELLOW);
        light2.setPosition(Constants.RED);

        //? change the step to the next or previous testing
        if(gamepad1.rightBumperWasPressed()){
            testStep++;
        } else if(gamepad1.leftBumperWasPressed()){
            testStep--;
        }

        //? Loops the steps
        while (testStep < 0) testStep+=4;
        while (testStep > 3) testStep-=4;

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
            case (3):
                bbTelemetry();
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
            intake.setPower(1.0);
        } else if(gamepad1.a){
            transfer.setPower(1.0);
        } else if(gamepad1.b){
            nectar.setPower(1.0);
        } else if(gamepad1.y){
            pollen.setPower(1.0);
        } else {
            intake.setPower(0.0);
            transfer.setPower(0.0);
            nectar.setPower(0.0);
            pollen.setPower(0.0);
        }
    }

    public void motorTelemetry(){
        telemetryBus.put("Testing Motors", "");
        telemetryBus.put("Intake Motor: ", intake.getPower());
        telemetryBus.put("Transfer Motor: ", transfer.getPower());
        telemetryBus.put("Nectar Motor: ", nectar.getPower());
        telemetryBus.put("Pollen Motor: ", pollen.getPower());
    }


    /* ----- SERVOS ----- */
    public void testServos(){
        if (gamepad1.xWasPressed()){
            x = !x;
            double pos = (x)? 1.0: 0.0;
            axon1.setPosition(pos);
            axon2.setPosition(pos);
        } else if(gamepad1.aWasPressed()){
            a = !a;
            double pos = (a)? Constants.MAX_HOOD: 0.0;
            hood1.setPosition(pos);
            hood2.setPosition(pos);
        } else if(gamepad1.bWasPressed()){
            b = !b;
            double pos = (b)? Constants.OPEN_GATE: Constants.CLOSED_GATE;
            nectarGate.setPosition(pos);
            pollenGate.setPosition(pos);
        } else if(gamepad1.yWasPressed()){
            y = !y;
            double pos = (y)? Constants.FLOWER_DOWN: Constants.FLOWER_UP;
            flower.setPosition(pos);
        }
    }

    public void servoTelemetry(){
        telemetryBus.put("Testing Servos", "");
        telemetryBus.put("Axon 1: ", axon1.getPosition());
        telemetryBus.put("Axon 2: ", axon2.getPosition());
        telemetryBus.put("Axon Encoder1: ", encoder1.getVoltage());
        telemetryBus.put("Axon Encoder2: ", encoder2.getVoltage());

        telemetryBus.put("Hood1: ", hood1.getPosition());
        telemetryBus.put("Hood2: ", hood2.getPosition());

        telemetryBus.put("Nectar Gate: ", nectarGate.getPosition());
        telemetryBus.put("Pollen Gate: ", pollenGate.getPosition());

        telemetryBus.put("Flower: ", flower.getPosition());
    }

    /* ----- BEAM BREAKS ----- */

    public void bbTelemetry(){
        telemetryBus.put("N0: ", n0.getState());
        telemetryBus.put("N1: ", n1.getState());
        telemetryBus.put("N2: ", n2.getState());
        telemetryBus.put("N3: ", n3.getState());

        telemetryBus.put("P0: ", p0.getState());
        telemetryBus.put("P1: ", p1.getState());
        telemetryBus.put("P2: ", p2.getState());
        telemetryBus.put("P3: ", p3.getState());
    }
}
