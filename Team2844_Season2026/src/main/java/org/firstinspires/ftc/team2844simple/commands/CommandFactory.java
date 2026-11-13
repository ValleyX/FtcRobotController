package org.firstinspires.ftc.team2844simple.commands;

import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.command.decorators.ParallelCommandGroup;

import org.firstinspires.ftc.team2844simple.commands.intake.IntakeCmd;
import org.firstinspires.ftc.team2844simple.commands.transfer.TransferCmd;
import org.firstinspires.ftc.team2844simple.helpers.Subsystems;
import org.firstinspires.ftc.team2844simple.subsystems.DriveSubsystem;
import org.firstinspires.ftc.team2844simple.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844simple.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.team2844simple.subsystems.TransferSubsystem;
import org.firstinspires.ftc.team2844simple.subsystems.VisionSubsystem;

public class CommandFactory {
    DriveSubsystem drive;
    IntakeSubsystem intake;
    ShooterSubsystem pollen;
    TransferSubsystem transfer;
    VisionSubsystem vision;

    public CommandFactory(Subsystems subsystems){
        drive = subsystems.drive;
        intake = subsystems.intake;
        pollen = subsystems.pollen;
        transfer = subsystems.transfer;
        vision = subsystems.vision;
    }

    public Command intakeAndTransfer(){
        return new ParallelCommandGroup(
                new IntakeCmd(intake),
                new TransferCmd(transfer)
        );
    }

    public static Command intakeAndTransfer(IntakeSubsystem intake, TransferSubsystem transfer){
        return new ParallelCommandGroup(
                new IntakeCmd(intake),
                new TransferCmd(transfer)
        );
    }

/*    public Command exampleIntake(){
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new IdleShootCmd(nectar),
                        new IdleShootCmd(pollen)
                ),
                new CloseGatesCmd(transfer),
                new IntakeCmd(intake),
                new TransferCmd(transfer)
        );
    }*/
}
