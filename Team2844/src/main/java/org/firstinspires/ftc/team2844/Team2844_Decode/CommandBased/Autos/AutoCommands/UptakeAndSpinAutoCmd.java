package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autos.AutoCommands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.ActivateIntakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.StopIntakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.StopTransferCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.TransferCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.StopSpinCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.UptakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.FallingEdgeTrigger;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterFeedSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SpindexerSubsystem;

import java.util.function.DoubleSupplier;

public class UptakeAndSpinAutoCmd extends ConditionalCommand {
    public UptakeAndSpinAutoCmd(ShooterFeedSubsystem shooterFeedSubsystem, KickSubsystem kickSubsystem, IntakeSubsystem intakeSubsystem, SpindexerSubsystem spindexerSubsystem, ShooterSubsystem shooterSubsystem, DoubleSupplier velocity){
        super(
                new ParallelCommandGroup(
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
                        new StopTransferCmd(shooterFeedSubsystem),
                        new StopSpinCmd(kickSubsystem),
                        new StopIntakeCmd(intakeSubsystem)
                ),
                () -> shooterSubsystem.inRange(velocity.getAsDouble()));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
