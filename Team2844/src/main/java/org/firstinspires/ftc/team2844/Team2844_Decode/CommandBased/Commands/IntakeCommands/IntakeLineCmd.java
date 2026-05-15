package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterFeedSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SpindexerSubsystem;

public class IntakeLineCmd extends CommandBase {
    ShooterFeedSubsystem shooterFeedSubsystem;
    IntakeSubsystem intakeSubsystem;
    SpindexerSubsystem spindexerSubsystem;
    KickSubsystem kickSubsystem;

    boolean ballInBeam;
    boolean topBroken;
    boolean bayOne;
    boolean hasBeenBayOne;
    ElapsedTime timer;

    public IntakeLineCmd(ShooterFeedSubsystem shooterFeedSubsystem, IntakeSubsystem intakeSubsystem, SpindexerSubsystem spindexerSubsystem, KickSubsystem kickSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        this.shooterFeedSubsystem = shooterFeedSubsystem;
        this.kickSubsystem = kickSubsystem;
        this.spindexerSubsystem = spindexerSubsystem;
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        addRequirements(intakeSubsystem, shooterFeedSubsystem, shooterFeedSubsystem);
    }

    @Override
    public void initialize() {
        hasBeenBayOne = false;
    }

    @Override
    public void execute() {
        boolean ballInBeam = intakeSubsystem.ballInBeam();
        boolean topBroken = shooterFeedSubsystem.topBroken();
        //boolean bayOne = spindexerSubsystem.ballInBayOne();

        if(ballInBeam){
            hasBeenBayOne = true;
        }

        if(!(ballInBeam && topBroken)){

            if(!topBroken){
//                new ParallelCommandGroup(new UptakeCmd(kickSubsystem), new TransferCmd(shooterFeedSubsystem));
               /* if(bayOne) {
                    if(ballInBeam){
                        intakeSubsystem.stop();
                    }

                    kickSubsystem.rotateKickerDown();
                } else {
                    if(hasBeenBayOne){
                        kickSubsystem.rotateKickerDownExtra();
                    } else {
                        intakeSubsystem.activate(Constants.INTAKE_SPEED);
                        kickSubsystem.rotateKickerDownIntake();
                    }
                }*/

                intakeSubsystem.activate(Constants.INTAKE_SPEED);
                kickSubsystem.rotateKickerDown();
                kickSubsystem.runKickerSpin();
                kickSubsystem.runSFeedForward();
                //shooterFeedSubsystem.runTFeedForward();
                timer.reset();
            } else {
//                new ParallelCommandGroup(new StopUptakeCmd(kickSubsystem), new StopTransferCmd(shooterFeedSubsystem));
                /*if(!bayOne){
                    if(shooterFeedSubsystem.tFeedBusy()){

                        kickSubsystem.rotateKickerDownExtra();
                        kickSubsystem.runKickerSpin();
                        kickSubsystem.runSFeedForward();

                        if(intakeSubsystem.ballInBeam()){
                            intakeSubsystem.stop();
                        } else {
                            intakeSubsystem.activate(Constants.INTAKE_SPEED);
                        }
                    } else {
                        kickSubsystem.rotateKickerDownIntake();
                        kickSubsystem.stopKickerSpin();
                        kickSubsystem.stopSFeed();
                        intakeSubsystem.activate(Constants.INTAKE_SPEED);
                    }
                } else {
                    intakeSubsystem.activate(Constants.INTAKE_SPEED);
                }
                //kickSubsystem.rotateKickerUp();
                //kickSubsystem.stopKickerSpin();
                //kickSubsystem.stopSFeed();


                if(timer.time() < 75) {
                    shooterFeedSubsystem.slowFeed();
                } else {
                    shooterFeedSubsystem.stopTFeed();
                }*/

                //if(!ballInBeam){
                    intakeSubsystem.activate(Constants.INTAKE_SPEED);
                //} else {
                //    intakeSubsystem.stop();
                //}
            }

        } else {
            //new ParallelCommandGroup(new StopUptakeCmd(kickSubsystem), new StopTransferCmd(shooterFeedSubsystem), new StopIntakeCmd(intakeSubsystem));
            intakeSubsystem.stop();
            kickSubsystem.rotateKickerUp();
            kickSubsystem.stopKickerSpin();
            kickSubsystem.stopSFeed();
            shooterFeedSubsystem.stopTFeed();
        }
    }

    @Override
    public boolean isFinished() {
        return (ballInBeam && topBroken);
    }
}
