package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.TeleOps;


import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.GamepadPair;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.RTPAxon;


// TeleOp test class for manual tuning and testing
@TeleOp(name = "Cont. Rotation Axon Test", group = "test")
public class CRAxonTest extends LinearOpMode {
    private CRServo turretAim;
    private AnalogInput axonIn;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //CRServo crservo = hardwareMap.crservo.get("rightHorizSlide");
        turretAim = hardwareMap.get(CRServo.class, Constants.CS2);
        //AnalogInput encoder = hardwareMap.get(AnalogInput.class, "rightHorizSlideEncoder");
        axonIn = hardwareMap.get(AnalogInput.class, Constants.CAI2);
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
        RTPAxon servo = new RTPAxon(turretAim, axonIn);

        servo.resetPID();
        servo.setKP(0.0105);
        servo.setKI(0.0002);
        servo.setKD(0.0005);
        //servo.setKP(0.015);
        //servo.setKI(0.0005);
        //servo.setKD(0.0025);
        servo.setRtp(true);


        waitForStart();

        while (!isStopRequested()) {
            gamepads.copyStates();
            servo.update();

            // Manual controls for target and PID tuning
            if (gamepads.isPressed(-1, "dpad_up")) {
                servo.changeTargetRotation(15);
            }
            if (gamepads.isPressed(-1, "dpad_down")) {
                servo.changeTargetRotation(-15);
            }
            if (gamepads.isPressed(-1, "a")) {
                servo.setTargetRotation(0);
            }
            /*A("a", "cross"),
                    B("b", "circle"),
                    X("x", "square"),
                    Y("y", "triangle"),*/
            if (gamepads.isPressed(-1, "y")) {
                servo.setKP(servo.getKP() + 0.001);
            }
            if (gamepads.isPressed(-1, "x")) {
                servo.setKP(Math.max(0, servo.getKP() - 0.001));
            }

            if (gamepads.isPressed(-1, "right_bumper")) {
                servo.setKI(servo.getKI() + 0.0001);
            }
            if (gamepads.isPressed(-1, "left_bumper")) {
                servo.setKI(Math.max(0, servo.getKI() - 0.0001));
            }

            if (gamepads.isPressed(-1, "dpad_left")) {
                servo.setKD(Math.max(0, servo.getKD() - 0.0001));
            }

            if (gamepads.isPressed(-1, "dpad_right")) {
                servo.setKD(Math.max(0, servo.getKD() + 0.0001));
            }

           /* if (gamepads.isPressed(-1, "touchpad")) {
                servo.setKP(0.015); //.005
                servo.setKI(0.0005); //.0001
                servo.setKD(0.0025); //?????
                servo.resetPID();
            }*/

            telemetry.addData("Starting angle", servo.STARTPOS);
            telemetry.addLine(servo.log());
            telemetry.addData("NTRY", servo.ntry);
            telemetry.addData("Angle", servo.getCurrentAngle());
            telemetry.update();
        }
    }
}
