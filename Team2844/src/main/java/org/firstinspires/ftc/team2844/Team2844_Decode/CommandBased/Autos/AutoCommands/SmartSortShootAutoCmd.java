package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autos.AutoCommands;

import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.FullAimToLLCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.ActivateIntakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.StopIntakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.StopTransferCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.TransferCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.VelocityShootCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.SlotCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.StopSpinCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.UptakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.FallingEdgeTrigger;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.DriveSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.SensorSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterFeedSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SpindexerSubsystem;

import java.util.function.DoubleSupplier;

public class SmartSortShootAutoCmd extends SequentialCommandGroup {
    SpindexerSubsystem spindexerSubsystem;
    KickSubsystem kickSubsystem;
    ShooterFeedSubsystem shooterFeedSubsystem;


    public SmartSortShootAutoCmd(ShooterSubsystem shooterSubsystem, ShooterFeedSubsystem shooterFeedSubsystem, SensorSubsystem sensorSubsystem, AimSubsystem aimSubsystem, SpindexerSubsystem spindexerSubsystem, KickSubsystem kickSubsystem, IntakeSubsystem intakeSubsystem, DriveSubsystem driveSubsystem, Vector2d vector, double heading, Telemetry telemetry) {
        DoubleSupplier velocity = () -> driveSubsystem.velocityLinReg(sensorSubsystem.getPipeline());

        this.spindexerSubsystem = spindexerSubsystem;
        this.kickSubsystem = kickSubsystem;
        this.shooterFeedSubsystem = shooterFeedSubsystem;
        int count = 0;


        addCommands(
                new ParallelCommandGroup(
                        //At the same time, aim the turret
                        new FullAimToLLCmd(aimSubsystem, sensorSubsystem, driveSubsystem),

                        //Also set the velocity to the amount based on distance from apriltag
                        new VelocityShootCmd(shooterSubsystem, velocity)
                ),

                new ConditionalCommand(
                        new SlotCmd(spindexerSubsystem, kickSubsystem, spindexerSubsystem.getSortPos()),
                        new InstantCommand(() -> spindexerSubsystem.runToSlot(spindexerSubsystem.getSlot())),
                        () -> spindexerSubsystem.fullSpindexer() && spindexerSubsystem.getSlot() != spindexerSubsystem.getSortPos()
                ),

                //if your at velocity, uptake and shoot, otherwise, don't
                new RepeatCommand(
                        new ConditionalCommand(
                                new ParallelCommandGroup(
                                        new TelemetryCmd(telemetry, "Repeat times: ", count++),
                                        new TransferCmd(shooterFeedSubsystem),
                                        new UptakeCmd(kickSubsystem),
                                        new ActivateIntakeCmd(intakeSubsystem),

                                        new ConditionalCommand(
                                                new InstantCommand(() -> spindexerSubsystem.runToSlot(spindexerSubsystem.getSlot())),
                                                new InstantCommand(() -> spindexerSubsystem.runToSlot(spindexerSubsystem.getSlot() - 1)),
                                                new FallingEdgeTrigger(spindexerSubsystem::ballInBayOne).asSupplier()
                                        )
                                ),
                                new ParallelCommandGroup(
                                        new TelemetryCmd(telemetry, "Repeat times: ", count++),
                                        new StopTransferCmd(shooterFeedSubsystem),
                                        new StopSpinCmd(kickSubsystem),
                                        new StopIntakeCmd(intakeSubsystem)
                                ),
                                () -> shooterSubsystem.inRange(velocity.getAsDouble())
                        )
                ).interruptOn(() -> spindexerSubsystem.empty() && !kickSubsystem.feedBusy() && !shooterFeedSubsystem.topBroken())
        );
    }
}

