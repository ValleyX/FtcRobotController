package org.firstinspires.ftc.team2844.Team2844_Decode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team2844.Team2844_Decode.Hardwares.RobotHardware;

@TeleOp(name = "Classify Prototype")
public class PrototypeClassify extends LinearOpMode {
    LinearOpMode opMode_;
    private ElapsedTime runtime = new ElapsedTime();
    RobotHardware robotHardware;

    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware = new RobotHardware(this);
        opMode_ = this;
        waitForStart();
        runtime.reset();
        double startTime = 0.0;
        float[] array = new float[3];

        while (opModeIsActive()) {
            array = robotHardware.getColors();
            telemetry.addData("Servo Pos", robotHardware.classSer.getPosition());
            telemetry.addData("Red", array[0]);
            telemetry.addData("Green", array[1]);
            telemetry.addData("Blue", array[2]);

            updateTelemetry(telemetry);

            if(array[0] > 0.01 && array[1] > 0.01 && array[2] > 0.01){
                startTime = System.currentTimeMillis();
                if(Math.max(array[1], array[2]) == array[1])
                {
                    robotHardware.moveServo(.35);
                } else if(Math.max(array[1], array[2]) == array[2]) {
                    robotHardware.moveServo(.65);
                }

            } else if(startTime - System.currentTimeMillis() < -1000){
                robotHardware.moveServo(.50);
            }
        }
    }
}
