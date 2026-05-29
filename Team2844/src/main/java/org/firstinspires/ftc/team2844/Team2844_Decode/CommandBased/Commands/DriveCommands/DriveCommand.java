package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.DriveCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {

    private DriveSubsystem driveSubsystem;
    private DoubleSupplier strafe, forward, turn;

    // heading supplier kept in signature so TeleOpBase doesn't need to change
    public DriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier turn, DoubleSupplier heading) {
        this.driveSubsystem = driveSubsystem;
        this.strafe = strafe;
        this.forward = forward;
        this.turn = turn;

        // heading unused — Pedro handles field-centric internally via its localizer

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        driveSubsystem.teleopDrive(-strafe.getAsDouble(), -forward.getAsDouble(), -turn.getAsDouble());
    }
}