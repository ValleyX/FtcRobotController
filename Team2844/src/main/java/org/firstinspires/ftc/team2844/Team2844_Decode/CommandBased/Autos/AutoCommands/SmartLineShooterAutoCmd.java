package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autos.AutoCommands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.AimHoodCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.FullAimToLLCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.ActivateIntakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.StopTransferCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.TransferCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.VelocityShootCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.SlotShootCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.StopSpinCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.StopUptakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.UptakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.DriveSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.SensorSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterFeedSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;

import java.util.function.DoubleSupplier;

public class SmartLineShooterAutoCmd extends ParallelCommandGroup {

    boolean empty;

    public SmartLineShooterAutoCmd(ShooterSubsystem shooterSubsystem, ShooterFeedSubsystem shooterFeedSubsystem, SensorSubsystem sensorSubsystem, AimSubsystem aimSubsystem, KickSubsystem kickSubsystem, IntakeSubsystem intakeSubsystem, DriveSubsystem driveSubsystem){

        empty = shooterFeedSubsystem.topBroken() && intakeSubsystem.ballInBeam();
        DoubleSupplier velocity = () -> driveSubsystem.velocityLinReg(sensorSubsystem.getPipeline());
        addCommands(
                //At the same time, aim the turret
                new FullAimToLLCmd(aimSubsystem, sensorSubsystem, driveSubsystem),

                //Also set the velocity to the amount based on distance from apriltag
                new VelocityShootCmd(shooterSubsystem, velocity),

                //if your at velocity, uptake and shoot, otherwise, don't
                new ConditionalCommand(
                        new ParallelCommandGroup(
                                new TransferCmd(shooterFeedSubsystem),
                                new UptakeCmd(kickSubsystem),
                                new ActivateIntakeCmd(intakeSubsystem)
                        ),
                        new ParallelCommandGroup(
                                new StopTransferCmd(shooterFeedSubsystem),
                                new StopSpinCmd(kickSubsystem),
                                new ActivateIntakeCmd(intakeSubsystem)
                        ),
                        shooterSubsystem::inRange
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
        return true;
    }
}
