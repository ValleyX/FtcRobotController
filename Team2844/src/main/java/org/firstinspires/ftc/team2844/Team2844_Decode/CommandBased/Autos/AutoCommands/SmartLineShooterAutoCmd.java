package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autos.AutoCommands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.AimHoodCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.FullAimToLLCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.ActivateIntakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.StopTransferCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.TransferCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.VelocityShootCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.SlotShootCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.StopSpinCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.StopUptakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.UptakeCmd;
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

public class SmartLineShooterAutoCmd extends SequentialCommandGroup {

    public SmartLineShooterAutoCmd(ShooterSubsystem shooterSubsystem, ShooterFeedSubsystem shooterFeedSubsystem,
                                   SensorSubsystem sensorSubsystem, AimSubsystem aimSubsystem, SpindexerSubsystem spindexerSubsystem,
                                   KickSubsystem kickSubsystem, IntakeSubsystem intakeSubsystem, DriveSubsystem driveSubsystem,
                                   Telemetry telemetry){

        DoubleSupplier velocity = () -> driveSubsystem.velocityLinReg(sensorSubsystem.getPipeline());
        ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        addCommands(
                new ParallelCommandGroup(
                //At the same time, aim the turret
                    new FullAimToLLCmd(aimSubsystem, sensorSubsystem, driveSubsystem),

                    //Also set the velocity to the amount based on distance from apriltag
                    new VelocityShootCmd(shooterSubsystem, velocity)
                )

                /*new RunCommand(() ->
                {
                    if(shooterSubsystem.inRange(velocity, shooterSubsystem::getVelocity).getAsBoolean()) {
                        telemetry.addData("In range: ", shooterSubsystem::getVelocity);
                        telemetry.update();

                        shooterFeedSubsystem.runTFeedForward();


                        if(spindexerSubsystem.ballInBayOne()){
                            intakeSubsystem.stop();
                            kickSubsystem.rotateKickerDown();
                            kickSubsystem.runSFeedForward();
                            kickSubsystem.runKickerSpin();
                            time.reset();
                        } else if(time.time(TimeUnit.MILLISECONDS) > 600 && intakeSubsystem.ballInBeam()){
                            kickSubsystem.rotateKickerDownIntake();
                            kickSubsystem.runSFeedForward();
                            kickSubsystem.runKickerSpin();
                            intakeSubsystem.activate(Constants.INTAKE_SPEED);
                        } else if (time.time(TimeUnit.MILLISECONDS) < 600 && time.time(TimeUnit.MILLISECONDS) > 300){
                            intakeSubsystem.stop();
                            kickSubsystem.rotateKickerDownExtra();
                            kickSubsystem.runSFeedForward();
                            kickSubsystem.runKickerSpin();
                        }

                        if(lastBall ||
                                (intakeSubsystem.ballInBeam() && !spindexerSubsystem.ballInBayOne() && !shooterFeedSubsystem.topBroken())){
                            if(lastBall){
                                if(shooterFeedSubsystem.topBroken()){
                                    lastBall = false;
                                }
                            } else {
                                intakeSubsystem.activate(Constants.INTAKE_SPEED);
                                lastBall = true;
                            }
                        }
                    } else  {
                        shooterFeedSubsystem.stopTFeed();
                        kickSubsystem.stopKickerSpin();
                        kickSubsystem.stopSFeed();
                        time.reset();
                    }
                }
                ).interruptOn(()-> !shooterFeedSubsystem.topBroken() && !spindexerSubsystem.ballInBayOne() &&
                        !intakeSubsystem.ballInBeam() && time.time(TimeUnit.MILLISECONDS) > 1200)*/


                //if your at velocity, uptake and shoot, otherwise, don't
                /*new ConditionalCommand(
                        new ParallelCommandGroup(
                                new TransferCmd(shooterFeedSubsystem),
                                new UptakeCmd(kickSubsystem),
                                new ActivateIntakeCmd(intakeSubsystem)
                        ),
                        new ParallelCommandGroup(
                                new StopTransferCmd(shooterFeedSubsystem),
                                new StopSpinCmd(kickSubsystem),
                                new ActivateIntakeCmd(intakeSubsystem)
                        ),
                        shooterSubsystem::inRange
                )*/
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
*/
}
