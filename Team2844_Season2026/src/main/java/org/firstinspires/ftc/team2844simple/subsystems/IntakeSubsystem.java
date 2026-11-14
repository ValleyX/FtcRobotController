package org.firstinspires.ftc.team2844simple.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vcs.valleylib.ftc.hardware.FtcSubsystem;

import org.firstinspires.ftc.team2844simple.helpers.Constants;

import java.util.concurrent.TimeUnit;

public class IntakeSubsystem extends FtcSubsystem {
    private final DcMotor intakeMotor;
    public IntakeSubsystem(HardwareMap hardwareMap) {
        super(hardwareMap);
        intakeMotor = hardwareMap.get(DcMotor.class, Constants.EHM0);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Run the Intake
     * */
    public void intake(){
        intakeMotor.setPower(Constants.INTAKE_SPEED);
    }
    public void intake(double speed){intakeMotor.setPower(speed);}

    /**
     * Stop the intake
     * */
    public void stopIntake(){
        intakeMotor.setPower(0.0);
    }

    /**
     * Run the intake backwards
     * */
    public void outtake(){
        intakeMotor.setPower(-Constants.INTAKE_SPEED);
    }
    @Override
    public void periodic() {}
}
