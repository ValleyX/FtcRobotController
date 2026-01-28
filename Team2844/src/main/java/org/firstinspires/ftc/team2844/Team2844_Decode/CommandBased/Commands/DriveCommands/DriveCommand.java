package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.DriveCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {

    private DriveSubsystem driveSubsystem;
    private DoubleSupplier strafe, forward, turn;

    public DriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier turn) {
        this.driveSubsystem = driveSubsystem;
        this.strafe = strafe;
        this.forward = forward;
        this.turn = turn;

        //this will prevent the drive subsystem from being used by another command
        addRequirements(driveSubsystem);
    }

    /* The normal process for a command is:
    @Override
      public void initialize() {
       // This will run ONCE when the Command is initialized
    }

    @Override
    public void execute() {
    //This will be run every period until isFinished is TRUE
    }

    @Override
    public boolean isFinished() {
    //This will be run after EXECUTE to check if the Command has finished
        return true;
    }

     @Override
    public boolean end(boolean interrupted) {
    //This is called once when isFinished is TRUE.  This is used to clean up the command (i.g. stop motors)
    }

     */
    @Override
    public void execute() {
        driveSubsystem.drive(strafe.getAsDouble(), forward.getAsDouble(), turn.getAsDouble());

    }
}
