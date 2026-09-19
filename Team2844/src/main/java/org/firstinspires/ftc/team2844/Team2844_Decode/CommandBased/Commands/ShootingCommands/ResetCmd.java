package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.IntakeLineCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.StopIntakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.StopUptakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.TransferCommands.StopTransferCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Subsystems;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterFeedSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SpindexerSubsystem;

public class ResetCmd extends ParallelCommandGroup {

    public ResetCmd(Subsystems subsystems){
        IntakeSubsystem intakeSubsystem = subsystems.intakeSubsystem;
        ShooterFeedSubsystem shooterFeedSubsystem = subsystems.shooterFeedSubsystem;
        KickSubsystem kickSubsystem = subsystems.kickSubsystem;
        addCommands(
                //new VelocityShootCmd(shooterSubsystem, () -> Constants.MIN_VELOCITY),
                new StopTransferCmd(shooterFeedSubsystem),
                new StopIntakeCmd(intakeSubsystem),
                //new NeutralAimCmd(aimSubsystem),
                new StopUptakeCmd(kickSubsystem)
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
