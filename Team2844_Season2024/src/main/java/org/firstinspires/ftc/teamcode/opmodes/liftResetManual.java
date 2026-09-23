package org.firstinspires.ftc.team12841.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team12841.Drivers.RobotHardware;
import org.firstinspires.ftc.team12841.commands.autocommands.SubToLength;
import org.firstinspires.ftc.team12841.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.team12841.subsystems.LiftSubsystem;

@TeleOp(name = "lift reset")

public class liftResetManual extends LinearOpMode {
    RobotHardware robot_;
    @Override
    public void runOpMode() throws InterruptedException {
        robot_ = new RobotHardware(this);
        LiftSubsystem m_lift = new LiftSubsystem(robot_.liftMotor, robot_.hangMotor, robot_.clawServo, robot_.hangServo, robot_.liftTouch, robot_.rightBase, robot_.leftBase);
        IntakeSubsystem m_intake = new IntakeSubsystem(robot_.subExtendMotor, robot_.sampleServo, robot_.intakeServo );
        SubToLength subLenghth = new SubToLength(m_intake,this);

        waitForStart();




        // run the scheduler
        //TODO; add telemetry into here if not already in commands or subsystems
        while (!isStopRequested() && opModeIsActive()) {



            //temporary
            if((-gamepad2.left_stick_y < 0) && m_lift.m_liftTouch.isPressed() /*&& robot_.liftMotor.getCurrentPosition() >= 0*/){
                m_lift.m_liftMotor.setPower(0);
            }else {
                m_lift.m_liftMotor.setPower(-gamepad2.left_stick_y);
            }
            if(gamepad1.a){

                m_intake.subExtendToPosition(0,1);
            }
            if(gamepad1.b){

                subLenghth.initialize(10);
            }


            if (gamepad1.left_bumper){
              //  robot_.intakeServo.setPosition(RobotHardware.INTAKE_SERVO_DOWN);
            }
            telemetry.addData("lift touch", robot_.liftTouch.isPressed());
            telemetry.addData("current pos", m_lift.m_liftMotor.getCurrentPosition());
            telemetry.addData("left base", robot_.leftBase.getValue());
            telemetry.update();

        }

    }
}
