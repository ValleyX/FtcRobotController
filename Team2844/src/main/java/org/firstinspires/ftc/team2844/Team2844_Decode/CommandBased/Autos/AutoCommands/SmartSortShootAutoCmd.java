package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autos.AutoCommands;

import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.google.blocks.ftcrobotcontroller.runtime.BlocksOpMode;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.AimHoodCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.FullAimToLLCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.ActivateIntakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.StopIntakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.StopTransferCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.TransferCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.VelocityShootCmd;
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

public class SmartSortShootAutoCmd extends SequentialCommandGroup {
    boolean ballInOne;
    boolean ballInTwo;
    boolean ballInThree;
    boolean ballInTFeed;
    boolean looped;


    SlotCmd previousSlot;

    public SmartSortShootAutoCmd(ShooterSubsystem shooterSubsystem, ShooterFeedSubsystem shooterFeedSubsystem, SensorSubsystem sensorSubsystem, AimSubsystem aimSubsystem, SpindexerSubsystem spindexerSubsystem, KickSubsystem kickSubsystem, IntakeSubsystem intakeSubsystem, DriveSubsystem driveSubsystem, Vector2d vector, double heading) {

        double velocity = driveSubsystem.velocityLinReg(sensorSubsystem.getPipeline());
        ballInOne = spindexerSubsystem.ballInBayOne();
        ballInTwo = spindexerSubsystem.ballInBayTwo();
        ballInThree = spindexerSubsystem.ballInBayThree();
        ballInTFeed = shooterFeedSubsystem.topBroken();
        boolean sorted = spindexerSubsystem.getSortPos() == spindexerSubsystem.getSlot();
        if(sorted && spindexerSubsystem.getSlot() != 0) {
            previousSlot = new SlotCmd(spindexerSubsystem, kickSubsystem, spindexerSubsystem.getSlot());
        } else {
            previousSlot = new SlotCmd(spindexerSubsystem, kickSubsystem, 0);
        }


        if (!sorted && !looped) {
            if (spindexerSubsystem.getSlot() == 0) {
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
                                        new UptakeCmd(kickSubsystem),
                                        new ActivateIntakeCmd(intakeSubsystem)
                                ),
                                new ParallelCommandGroup(
                                        new StopTransferCmd(shooterFeedSubsystem),
                                        new StopSpinCmd(kickSubsystem),
                                        new StopIntakeCmd(intakeSubsystem)
                                ),
                                shooterSubsystem::inRange
                        )
                );
            }
        } else {
            if (previousSlot.isFinished() && !ballInOne && spindexerSubsystem.getSlot() != 0) {
                addCommands(
                        new ParallelCommandGroup(
                                //At the same time, aim the turret
                                new FullAimToLLAutoCmd(aimSubsystem, sensorSubsystem, driveSubsystem, vector, heading),

                                //Also set the velocity to the amount based on distance from apriltag
                                new VelocityShootCmd(shooterSubsystem, velocity),

                                //go to the slot that sorts it
                                new SlotCmd(spindexerSubsystem, kickSubsystem, spindexerSubsystem.getSlot())
                        ),

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
                                        new StopIntakeCmd(intakeSubsystem)
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
                                new SlotCmd(spindexerSubsystem, kickSubsystem, spindexerSubsystem.getSlot())
                        ),

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
                                        new StopIntakeCmd(intakeSubsystem)
                                ),
                                shooterSubsystem::inRange
                        )
                );
            }
        }
    }

    @Override
    public void initialize() {
        looped = false;
    }

    @Override
    public boolean isFinished() {
        return !(ballInOne || ballInTwo || ballInThree || ballInTFeed || !previousSlot.isScheduled());
    }

    @Override
    public void end(boolean interrupted) {
        looped = false;
    }
}

