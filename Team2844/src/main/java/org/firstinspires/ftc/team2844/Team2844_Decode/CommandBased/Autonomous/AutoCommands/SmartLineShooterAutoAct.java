package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.AutoCommands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.FullAimToLLCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.ActivateIntakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.VelocityShootCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
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

public class SmartLineShooterAutoAct implements Action {

    SpindexerSubsystem spindexerSubsystem;
    ShooterFeedSubsystem shooterFeedSubsystem;
    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;
    SensorSubsystem sensorSubsystem;
    AimSubsystem aimSubsystem;
    KickSubsystem kickSubsystem;
    DriveSubsystem driveSubsystem;

    DoubleSupplier velocity;
    ElapsedTime timer;

    boolean init;
    public SmartLineShooterAutoAct(ShooterSubsystem shooterSubsystem, ShooterFeedSubsystem shooterFeedSubsystem,
                                   SensorSubsystem sensorSubsystem, AimSubsystem aimSubsystem, SpindexerSubsystem spindexerSubsystem,
                                   KickSubsystem kickSubsystem, IntakeSubsystem intakeSubsystem, DriveSubsystem driveSubsystem,
                                   Telemetry telemetry){

        this.spindexerSubsystem = spindexerSubsystem;
        this.shooterFeedSubsystem = shooterFeedSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.sensorSubsystem = sensorSubsystem;
        this.aimSubsystem = aimSubsystem;
        this.kickSubsystem = kickSubsystem;
        this.driveSubsystem = driveSubsystem;

        timer = new ElapsedTime();
        timer.reset();

        init = true;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if(init){
            timer.reset();
            init = false;
        }

        velocity = () -> driveSubsystem.velocityLinReg(sensorSubsystem.getPipeline());

        new SequentialAction(
                new ParallelAction(
                        new FullAimToLLAct(aimSubsystem, sensorSubsystem, driveSubsystem),
                        new CommandAction(new VelocityShootCmd(shooterSubsystem, velocity))
                ),
                new ParallelAction(
                        new FullAimToLLAct(aimSubsystem, sensorSubsystem, driveSubsystem, false),
                        new CommandAction(new VelocityShootCmd(shooterSubsystem, velocity)),
                        //new CommandAction(new ActivateIntakeCmd(intakeSubsystem)),
                        new InRange(shooterFeedSubsystem, kickSubsystem, shooterSubsystem, intakeSubsystem)
                )
        ).run(telemetryPacket);

        if(timer.time(TimeUnit.MILLISECONDS) < Constants.SHOOTER_TIMEOUT) {
            return true;
        } else if(timer.time(TimeUnit.MILLISECONDS) < Constants.SHOOTER_TIMEOUT + 1000) {
            new FullTransferAct(shooterFeedSubsystem, intakeSubsystem, kickSubsystem).run(telemetryPacket);
            return true;
        } else{
            return (shooterFeedSubsystem.topBroken() || intakeSubsystem.ballInBeam());
        }

    }
}
