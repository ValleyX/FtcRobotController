package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autos.AutoCommands;

import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.FullAimToLLCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.ActivateIntakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.StopIntakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.StopTransferCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.TransferCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.VelocityShootCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.SlotCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.StopSpinCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.StopUptakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.UptakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.FallingEdgeTrigger;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.DriveSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.SensorSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterFeedSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SpindexerSubsystem;

import java.util.concurrent.TimeUnit;
import java.util.function.DoubleSupplier;

public class SmartSortShootAutoCmd extends SequentialCommandGroup {


    public SmartSortShootAutoCmd(ShooterSubsystem shooterSubsystem, ShooterFeedSubsystem shooterFeedSubsystem, SensorSubsystem sensorSubsystem, AimSubsystem aimSubsystem, SpindexerSubsystem spindexerSubsystem, KickSubsystem kickSubsystem, IntakeSubsystem intakeSubsystem, DriveSubsystem driveSubsystem, Vector2d vector, double heading, Telemetry telemetry) {
        DoubleSupplier velocity = () -> driveSubsystem.velocityLinReg(sensorSubsystem.getPipeline());
        ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime stuckTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


        addCommands(
                new ParallelCommandGroup(
                        new VelocityShootCmd(shooterSubsystem, velocity),

                        //At the same time, aim the turret
                        new FullAimToLLCmd(aimSubsystem, sensorSubsystem, driveSubsystem),

                        new TelemetryCmd(telemetry, "In ParallelDeadlineGroup", velocity.getAsDouble()),
                        new TelemetryCmd(telemetry, "in range", shooterSubsystem.inRange(velocity, shooterSubsystem::getVelocity).getAsBoolean())
                ),

                new TelemetryCmd(telemetry, "Out of parallel: ", true),

                new SlotCmd(spindexerSubsystem, kickSubsystem, spindexerSubsystem.getSortPos()),

                new TelemetryCmd(telemetry, "Past SlotCmd: ", true),


                //if your at velocity, uptake and shoot, otherwise, don't

                new RunCommand(() -> {
                    telemetry.addData("In execute", shooterSubsystem::getVelocity);
                    telemetry.addData("Empty? ", spindexerSubsystem::empty);
                    telemetry.update();
                    boolean bayOne = spindexerSubsystem.ballInBayOne();
                    if (shooterSubsystem.inRange(velocity, shooterSubsystem::getVelocity).getAsBoolean()) {
                        telemetry.addData("In range: ", shooterSubsystem::getVelocity);
                        telemetry.update();


                        shooterFeedSubsystem.runTFeedForward();

                        if(bayOne){
                            kickSubsystem.runSFeedForward();
                            if (spindexerSubsystem.getSlot() != 0) {
                                kickSubsystem.rotateKickerDown();
                                kickSubsystem.runKickerSpin();
                            } else {
                                kickSubsystem.rotateKickerDownExtra();
                                kickSubsystem.runKickerSpin();
                            }
                            time.reset();
                        } else if(time.time(TimeUnit.MILLISECONDS) > 450) {
                            spindexerSubsystem.runToShootSlot(spindexerSubsystem.getSlot() - 1);

                            if(spindexerSubsystem.empty() && !shooterFeedSubsystem.topBroken()) {
                                kickSubsystem.stopSFeed();
                                kickSubsystem.stopKickerSpin();
                                kickSubsystem.rotateKickerUp();
                            }
                            time.reset();
                        }

                    } else {
                        telemetry.addData("Not in range: ", shooterSubsystem::getVelocity);
                        telemetry.update();
                        shooterFeedSubsystem.stopTFeed();
                        kickSubsystem.stopSFeed();
                        kickSubsystem.stopKickerSpin();
                    }
                }).interruptOn(() ->  (spindexerSubsystem.empty()
                        && !kickSubsystem.feedBusy() && !shooterFeedSubsystem.topBroken())),
                new StopUptakeCmd(kickSubsystem)
        );
    }
}
