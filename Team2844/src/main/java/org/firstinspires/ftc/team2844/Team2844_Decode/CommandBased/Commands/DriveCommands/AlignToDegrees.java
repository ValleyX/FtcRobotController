package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.DriveCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class AlignToDegrees extends CommandBase {
    DriveSubsystem driveSubsystem;
    double speed;
    double heading;
    DoubleSupplier tx;

    public AlignToDegrees(DriveSubsystem driveSubsystem, DoubleSupplier tx, double speed) {
        this.driveSubsystem = driveSubsystem;
        this.tx = tx;
        this.speed = speed;
    }

    @Override
    public void execute() {
        /*
        double oppDeg;
        double mult;
        heading = driveSubsystem.getRobotHeading();
        double degrees = this.degrees.getAsDouble();*/
        double tx = this.tx.getAsDouble();
        double power = 0.0;
        if(tx != Constants.NO_LL)
            power = -(tx * Constants.ALIGN_GAIN * speed);
        driveSubsystem.setAlignPower(-power, -power, power, power);
/*
        mult = Math.abs(Math.abs(degrees) - Math.abs(heading)) * Constants.ALIGN_GAIN;
        if (!(degrees - Constants.TURRET_THRESHHOLD < heading && heading < degrees + Constants.TURRET_THRESHHOLD)) {
            if (degrees > 0) {
                oppDeg = degrees - 180;
                if (oppDeg < heading && heading < degrees) {
                    driveSubsystem.setAlignPower(Math.min(-speed * mult, 1), Math.min(-speed * mult, 1), Math.min(speed * mult, 1), Math.min(speed * mult, 1));
                } else {
                    driveSubsystem.setAlignPower(Math.min(speed * mult, 1), Math.min(speed * mult, 1), Math.min(-speed * mult, 1), Math.min(-speed * mult, 1));
                }
            } else {
                oppDeg = degrees + 180;
                if (degrees < heading && heading < oppDeg) {
                    driveSubsystem.setAlignPower(Math.min(speed * mult, 1), Math.min(speed * mult, 1), Math.min(-speed * mult, 1), Math.min(-speed * mult, 1));
                } else {
                    driveSubsystem.setAlignPower(Math.min(-speed * mult, 1), Math.min(-speed * mult, 1), Math.min(speed * mult, 1), Math.min(speed * mult, 1));
                }
            }
        } else {
            driveSubsystem.setAlignPower(0, 0, 0, 0);
        }
        */
    }

    @Override
    public boolean isFinished() {
        return (tx.getAsDouble() - Constants.TURRET_THRESHHOLD < heading && heading < tx.getAsDouble() + Constants.TURRET_THRESHHOLD);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setAlignPower(0.0,0.0,0.0,0.0);
    }
}
