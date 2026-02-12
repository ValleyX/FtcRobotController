package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.AimHoodCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.FullAimToLLCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.ActivateIntakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.StopUptakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.UptakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.SensorSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterFeedSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;

public class SmartLineShooterCmd extends ParallelCommandGroup {

    /*ShooterSubsystem shooterSubsystem;
    SensorSubsystem sensorSubsystem;

    AimSubsystem aimSubsystem;
    KickSubsystem kickSubsystem;
    ShooterFeedSubsystem shooterFeedSubsystem;
    SpindexerSubsystem spindexerSubsystem;*/

    public SmartLineShooterCmd(ShooterSubsystem shooterSubsystem, ShooterFeedSubsystem shooterFeedSubsystem, SensorSubsystem sensorSubsystem, AimSubsystem aimSubsystem, KickSubsystem kickSubsystem, IntakeSubsystem intakeSubsystem){
        /*this.shooterSubsystem = shooterSubsystem;
        this.sensorSubsystem = sensorSubsystem;
        this.aimSubsystem = aimSubsystem;
        this.kickSubsystem = kickSubsystem;
        this.shooterFeedSubsystem = shooterFeedSubsystem;
        this.spindexerSubsystem = spindexerSubsystem;
        addRequirements(shooterSubsystem);*/

        double velocity = sensorSubsystem.velocityLinReg();
        addCommands(
                //At the same time, aim the turret
                new FullAimToLLCmd(aimSubsystem, sensorSubsystem),
                new AimHoodCmd(aimSubsystem, sensorSubsystem),

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
                                new StopUptakeCmd(kickSubsystem),
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

    @Override
    public boolean isFinished() {
        //return spindexerSubsystem.empty() && !shooterFeedSubsystem.topBroken();
        return true;
    }*/
}
