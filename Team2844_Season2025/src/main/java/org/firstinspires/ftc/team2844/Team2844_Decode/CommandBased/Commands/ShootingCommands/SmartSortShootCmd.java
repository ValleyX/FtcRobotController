package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.FullAimToLLCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.MoveArtifactShootCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.TransferCommands.UptakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.TransferCommands.UptakeExtraCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.TransferCommands.UptakeLessCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.TransferCommands.StopTransferCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.TransferCommands.TransferCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.DriveSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.SensorSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterFeedSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SpindexerSubsystem;

import java.util.function.DoubleSupplier;

public class SmartSortShootCmd extends SequentialCommandGroup {
    public SmartSortShootCmd(ShooterSubsystem shooterSubsystem, ShooterFeedSubsystem shooterFeedSubsystem, SensorSubsystem sensorSubsystem, AimSubsystem aimSubsystem, SpindexerSubsystem spindexerSubsystem, KickSubsystem kickSubsystem, DriveSubsystem driveSubsystem){
        DoubleSupplier velocity = () -> driveSubsystem.velocityLinReg(sensorSubsystem.getPipeline());
        //InstantCommand currentSlot = new InstantCommand(() ->spindexerSubsystem.runToSlot(spindexerSubsystem.getSlot()), spindexerSubsystem);


            addCommands(
                    new ParallelCommandGroup(
                            //At the same time, aim the turret
                            new FullAimToLLCmd(aimSubsystem, sensorSubsystem, driveSubsystem),

                            //Also set the velocity to the amount based on distance from apriltag
                            new VelocityShootCmd(shooterSubsystem, velocity),


                            //if your at velocity, uptake and shoot, otherwise, don't
                            new ConditionalCommand(
                                    new ParallelCommandGroup(
                                            new ConditionalCommand(
                                                    new ConditionalCommand(
                                                            new SequentialCommandGroup(new UptakeLessCmd(kickSubsystem), new MoveArtifactShootCmd(spindexerSubsystem).withTimeout(2000)),
                                                            new ConditionalCommand(
                                                                    new UptakeExtraCmd(kickSubsystem),
                                                                    new UptakeCmd(kickSubsystem),
                                                                    spindexerSubsystem::empty
                                                            ),
                                                            spindexerSubsystem::bayOneReady
                                                    ),
                                                    new InstantCommand(),
                                                    shooterFeedSubsystem::topBroken
                                            ),
                                            new TransferCmd(shooterFeedSubsystem)
                                    ),
                                    new ParallelCommandGroup(new StopTransferCmd(shooterFeedSubsystem), new UptakeCmd(kickSubsystem)),
                                    shooterSubsystem::inRange
                            )
                            //go to the slot that sorts it
                            //new SlotCmd(spindexerSubsystem, kickSubsystem, spindexerSubsystem.getSortPos())
                    )
            );
    }
}
