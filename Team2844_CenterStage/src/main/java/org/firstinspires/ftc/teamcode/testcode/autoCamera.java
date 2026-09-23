package org.firstinspires.ftc.team12841.testcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name="Test camera auto", group = "Concept")
public class autoCamera extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardwareTestVersion robot = new RobotHardwareTestVersion(this,true);//RobotHardwareTestVersion(this, true);
        //int count = 0;
        while(opModeInInit()){
            //telemetry.addData("red value", robotHardware_.pipeline.avgLeftRed);
            //telemetry.addData("blue value", robotHardware_.pipeline.avgLeftBlue);

            telemetry.addData("r value", robot.pipeline.avgR);
            telemetry.addData("b value", robot.pipeline.avgB);
            telemetry.addData("r2 value", robot.pipeline.avg2R);
            telemetry.addData("b2 value", robot.pipeline.avg2B);
            telemetry.addData("r3 value", robot.pipeline.avg3R);
            telemetry.addData("b3 value", robot.pipeline.avg3B);


            telemetry.addData("alpha position", robot.pipeline.position);
            //telemetry.addData("test value", count++);

            telemetry.update();

            //alphaColor = robot.pipeline.color;
        }
    }
}
