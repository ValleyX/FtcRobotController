package org.firstinspires.ftc.team12841.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team12841.RobotHardware;

@Disabled
@TeleOp(name = "MotorTester", group = "MAIN")
public class MotorTester extends OpMode {

    /* ===================== HARDWARE ===================== */

    private RobotHardware robot;

    @Override
    public void init() {
        robot = new RobotHardware(this);

        telemetry.addLine("Init OK — warming localization");
        telemetry.update();
    }

    /* ===================== LOOP ===================== */

    @Override
    public void loop(){

        if(gamepad1.a){
            robot.lf.setPower(1); // Correct but reversed
        }else if(gamepad1.b){
            robot.lb.setPower(1); // Correct but Reversed
        }else if(gamepad1.x){
            robot.rf.setPower(1);
        }else if(gamepad1.y){
            robot.rb.setPower(1);
        } else {
            robot.lf.setPower(0);
            robot.lb.setPower(0);
            robot.rf.setPower(0);
            robot.rb.setPower(0);
        }

}}
