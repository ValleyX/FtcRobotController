package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.AimHoodCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.FullAimToLLCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.ActivateIntakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.StopIntakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.SlotCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.StopSpinCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.UptakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.DriveSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.SensorSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterFeedSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SpindexerSubsystem;

public class SmartSortShootCmd extends SequentialCommandGroup {
    public SmartSortShootCmd(ShooterSubsystem shooterSubsystem, ShooterFeedSubsystem shooterFeedSubsystem, SensorSubsystem sensorSubsystem, AimSubsystem aimSubsystem, SpindexerSubsystem spindexerSubsystem, KickSubsystem kickSubsystem, DriveSubsystem driveSubsystem){
        double velocity = driveSubsystem.velocityLinReg(sensorSubsystem.getPipeline());
        SlotCmd previousSlot = new SlotCmd(spindexerSubsystem, kickSubsystem, spindexerSubsystem.getSlot() - 1);

        if(!previousSlot.isScheduled() && !spindexerSubsystem.ballInBayOne() && spindexerSubsystem.getSlot() != 0) {
            addCommands(
                    new ParallelCommandGroup(
                            //At the same time, aim the turret
                            new FullAimToLLCmd(aimSubsystem, sensorSubsystem, driveSubsystem),

                            //Also set the velocity to the amount based on distance from apriltag
                            new VelocityShootCmd(shooterSubsystem, velocity),

                            //go to the slot that sorts it
                            new SlotCmd(spindexerSubsystem, kickSubsystem, spindexerSubsystem.getSortPos())
                    ),

                    //if your at velocity, uptake and shoot, otherwise, don't
                    new ConditionalCommand(
                            new ParallelCommandGroup(
                                    new TransferCmd(shooterFeedSubsystem),
                                    new UptakeCmd(kickSubsystem)
                            ),
                            new ParallelCommandGroup(
                                    new StopTransferCmd(shooterFeedSubsystem),
                                    new StopSpinCmd(kickSubsystem)
                            ),
                            shooterSubsystem::inRange
                    ),
                    previousSlot
            );
        } else {
            addCommands(
                    new ParallelCommandGroup(
                            //At the same time, aim the turret
                            new FullAimToLLCmd(aimSubsystem, sensorSubsystem, driveSubsystem),

                            //Also set the velocity to the amount based on distance from apriltag
                            new VelocityShootCmd(shooterSubsystem, velocity),

                            //go to the slot that sorts it
                            new SlotCmd(spindexerSubsystem, kickSubsystem, spindexerSubsystem.getSortPos())
                    ),

                    //if your at velocity, uptake and shoot, otherwise, don't
                    new ConditionalCommand(
                            new ParallelCommandGroup(
                                    new TransferCmd(shooterFeedSubsystem),
                                    new UptakeCmd(kickSubsystem)
                            ),
                            new ParallelCommandGroup(
                                    new StopTransferCmd(shooterFeedSubsystem),
                                    new StopSpinCmd(kickSubsystem)
                            ),
                            shooterSubsystem::inRange
                    )
            );
        }
    }
}
