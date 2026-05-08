package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.AutoCommands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.FullAimToLLCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.ActivateIntakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.StopIntakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.StopTransferCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.TransferCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.VelocityShootCmd;
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

public class SmartLineShooterAutoCmd extends SequentialCommandGroup {

    SpindexerSubsystem spindexerSubsystem;
    ShooterFeedSubsystem shooterFeedSubsystem;
    IntakeSubsystem intakeSubsystem;
    public SmartLineShooterAutoCmd(ShooterSubsystem shooterSubsystem, ShooterFeedSubsystem shooterFeedSubsystem,
                                   SensorSubsystem sensorSubsystem, AimSubsystem aimSubsystem, SpindexerSubsystem spindexerSubsystem,
                                   KickSubsystem kickSubsystem, IntakeSubsystem intakeSubsystem, DriveSubsystem driveSubsystem,
                                   Telemetry telemetry){
        this.spindexerSubsystem = spindexerSubsystem;
        this.shooterFeedSubsystem = shooterFeedSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        DoubleSupplier velocity = () -> driveSubsystem.velocityLinReg(sensorSubsystem.getPipeline());
        //ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

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
        return (spindexerSubsystem.empty() && shooterFeedSubsystem.topBroken() && intakeSubsystem.ballInBeam());
    }
}
