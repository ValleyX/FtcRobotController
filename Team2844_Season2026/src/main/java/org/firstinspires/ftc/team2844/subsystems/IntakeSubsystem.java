package org.firstinspires.ftc.team2844.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vcs.valleylib.ftc.hardware.FtcSubsystem;

import org.firstinspires.ftc.team2844.helpers.Constants;

public class IntakeSubsystem extends FtcSubsystem {
    DcMotor intakeMotor;
    public IntakeSubsystem(HardwareMap hardwareMap) {
        super(hardwareMap);

        intakeMotor = hardwareMap.get(DcMotor.class, Constants.EHM0);
    }
}
