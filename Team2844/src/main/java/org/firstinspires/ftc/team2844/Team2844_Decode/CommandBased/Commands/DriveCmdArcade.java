package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.TankDriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveCmdArcade extends CommandBase {

    private TankDriveSubsystem tankDriveSubsystem;
    private DoubleSupplier forwardSpeed, turnSpeed;

    public DriveCmdArcade(TankDriveSubsystem tankDriveSubsystem, DoubleSupplier forwardSpeed, DoubleSupplier turnSpeed) {
        this.tankDriveSubsystem = tankDriveSubsystem;
        this.forwardSpeed = forwardSpeed;
        this.turnSpeed = turnSpeed;

        //this will prevent the drive subsystem from being used by another command
        addRequirements(tankDriveSubsystem);
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
        tankDriveSubsystem.arcadeDrive(forwardSpeed.getAsDouble(), turnSpeed.getAsDouble());
    }

}
