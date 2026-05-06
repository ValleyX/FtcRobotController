package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.FullAimToLLCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.ActivateIntakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.StopIntakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.StopUptakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.UptakeShootCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.DriveSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.SensorSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterFeedSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SpindexerSubsystem;

import java.util.function.DoubleSupplier;

public class LineShooterRegCmd extends SequentialCommandGroup {
    public LineShooterRegCmd(ShooterSubsystem shooterSubsystem, ShooterFeedSubsystem shooterFeedSubsystem,
                             SensorSubsystem sensorSubsystem, AimSubsystem aimSubsystem, KickSubsystem kickSubsystem,
                             IntakeSubsystem intakeSubsystem, DriveSubsystem driveSubsystem,
                             SpindexerSubsystem spindexerSubsystem, DoubleSupplier velocity){
        addCommands(
                new ParallelCommandGroup(
                        //At the same time, aim the turret
                        new FullAimToLLCmd(aimSubsystem, sensorSubsystem, driveSubsystem),

                        //Also set the velocity to the amount based on distance from apriltag
                        new VelocityShootCmd(shooterSubsystem, velocity),

                        new ConditionalCommand(
                                new StopIntakeCmd(intakeSubsystem),
                                new ActivateIntakeCmd(intakeSubsystem),
                                spindexerSubsystem::ballInBayOne
                        ),

                        //if your at velocity, uptake and shoot, otherwise, don't
                        new ConditionalCommand(
                                new ParallelCommandGroup(
                                        new TransferCmd(shooterFeedSubsystem),
                                        new ConditionalCommand(
                                                new StopUptakeCmd(kickSubsystem),
                                                new SequentialCommandGroup(new UptakeShootCmd(kickSubsystem, spindexerSubsystem, shooterFeedSubsystem)),
                                                shooterFeedSubsystem::topBroken
                                        )

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

    @Override
    public boolean isFinished() {
        return true;
    }

}
