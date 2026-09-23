package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterSubsystem;

public class SetVeloPIDS extends CommandBase {
    ShooterSubsystem shooterSubsystem;
    HardwareMap hardwareMap;
    boolean finished;
    public SetVeloPIDS(ShooterSubsystem shooterSubsystem, HardwareMap hardwareMap, boolean finished){
        this.shooterSubsystem = shooterSubsystem;
        this.hardwareMap = hardwareMap;
        this.finished = finished;
    }

    @Override
    public void execute() {
        shooterSubsystem.setPIDs(
                Constants.P_GAIN,
                Constants.I_GAIN,
                Constants.D_GAIN
        );
        shooterSubsystem.setFeedForward(
                Constants.VEL_KS,
                Constants.VEL_KV * (12.0/hardwareMap.voltageSensor.iterator().next().getVoltage())
        );
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
