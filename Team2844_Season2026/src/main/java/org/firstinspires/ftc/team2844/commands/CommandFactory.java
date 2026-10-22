package org.firstinspires.ftc.team2844.commands;

import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.command.decorators.ParallelCommandGroup;

import org.firstinspires.ftc.team2844.commands.intake.IntakeCmd;
import org.firstinspires.ftc.team2844.commands.transfer.TransferCmd;
import org.firstinspires.ftc.team2844.helpers.Subsystems;
import org.firstinspires.ftc.team2844.subsystems.DriveSubsystem;
import org.firstinspires.ftc.team2844.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.subsystems.LightSubsystem;
import org.firstinspires.ftc.team2844.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.team2844.subsystems.TransferSubsystem;
import org.firstinspires.ftc.team2844.subsystems.TurretSubsystem;
import org.firstinspires.ftc.team2844.subsystems.VisionSubsystem;

public class CommandFactory {
    DriveSubsystem drive;
    IntakeSubsystem intake;
    LightSubsystem light;
    ShooterSubsystem nectar;
    ShooterSubsystem pollen;
    TransferSubsystem transfer;
    TurretSubsystem turret;
    VisionSubsystem vision;

    public CommandFactory(Subsystems subsystems){
        drive = subsystems.drive;
        intake = subsystems.intake;
        light = subsystems.light;
        nectar = subsystems.nectar;
        pollen = subsystems.pollen;
        transfer = subsystems.transfer;
        turret = subsystems.turret;
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
