package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.FullAimToLLCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.ActivateIntakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.StopUptakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.TransferCommands.UptakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.TransferCommands.UptakeExtraCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.TransferCommands.StopTransferCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.TransferCommands.TransferCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Subsystems;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.DriveSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.SensorSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterFeedSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SpindexerSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SmartLineShooterCmd extends SequentialCommandGroup {

    public SmartLineShooterCmd(Subsystems subsystems, BooleanSupplier manualAim){

        ShooterSubsystem shooterSubsystem = subsystems.shooterSubsystem;
        ShooterFeedSubsystem shooterFeedSubsystem = subsystems.shooterFeedSubsystem;
        SensorSubsystem sensorSubsystem = subsystems.sensorSubsystem;
        AimSubsystem aimSubsystem = subsystems.aimSubsystem;
        KickSubsystem kickSubsystem = subsystems.kickSubsystem;
        IntakeSubsystem intakeSubsystem = subsystems.intakeSubsystem;
        DriveSubsystem driveSubsystem = subsystems.mecDriveSubsystem;

        DoubleSupplier velocity = () -> driveSubsystem.velocityLinReg(sensorSubsystem.getPipeline());
        addRequirements(shooterSubsystem, shooterFeedSubsystem, aimSubsystem, kickSubsystem, intakeSubsystem);

        addCommands(
                new ParallelCommandGroup(
                        new VelocityShootCmd(shooterSubsystem, velocity),
                        new FullAimToLLCmd(aimSubsystem, sensorSubsystem, driveSubsystem, manualAim, true)
                ),
                new ParallelCommandGroup(
                        //At the same time, aim the turret
                        new FullAimToLLCmd(aimSubsystem, sensorSubsystem, driveSubsystem, manualAim, true),

                        //Also set the velocity to the amount based on distance from apriltag
                        new VelocityShootCmd(shooterSubsystem, velocity),

                        //new ConditionalCommand(
                        //        new StopIntakeCmd(intakeSubsystem),
                        //        new ActivateIntakeCmd(intakeSubsystem),
                                //spindexerSubsystem::ballInBayOne
                        //),

                        //if your at velocity, uptake and shoot, otherwise, don't
                        new ConditionalCommand(
                                new ParallelCommandGroup(
                                        new TransferCmd(shooterFeedSubsystem),
                                        new ConditionalCommand(
                                                new UptakeExtraCmd(kickSubsystem),
                                                new UptakeCmd(kickSubsystem),
                                                shooterFeedSubsystem::topBroken
                                        ),
                                        new ActivateIntakeCmd(intakeSubsystem)

                                ),
                                new ParallelCommandGroup(
                                        new StopTransferCmd(shooterFeedSubsystem),
                                        new StopUptakeCmd(kickSubsystem)
                                ),
                                shooterSubsystem::inRange
                        )
                )
        );
    }

    /*@Override
    public void execute(){
        new FullAimToLLCmd(aimSubsystem, sensorSubsystem);

        double velo = sensorSubsystem.velocityLinReg();
        shooterSubsystem.setVelocity(velo);
        if(shooterSubsystem.inRange(velo)){
            new ParallelCommandGroup(new UptakeCmd(kickSubsystem), new TransferCmd(shooterFeedSubsystem));
        }

    }
*/
    @Override
    public boolean isFinished() {
        //return spindexerSubsystem.empty() && !shooterFeedSubsystem.topBroken();
        return true;
    }
}
