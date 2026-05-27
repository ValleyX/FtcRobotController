package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.Roadrunner.Drawing;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.AimTurretCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.MoveHoodNegative;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.MoveHoodPositive;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.MoveTurretNegative;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.MoveTurretPositive;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.DriveCommands.DriveCommand;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.DriveCommands.ResetImuCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.DriveCommands.ResetPoseCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.IntakeLineCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.IntakeSortCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.StopIntakeLineCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.StopIntakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.ResetCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.TransferCommands.StopTransferCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.TransferCommands.TransferCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.StopUptakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.TransferCommands.UptakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.SavedVars;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Subsystems;

@TeleOp(name = "Zeroing OpMode")
public class ZeroingOp extends CommandOpMode {
    /* ------------------- Command Declarations ------------------- */

    private DriveCommand mecDriveCmd;
    StopIntakeCmd stopIntakeCmd;
    IntakeSortCmd runIntakeSortCmd;
    UptakeCmd uptakeCmd;
    AimTurretCmd neutralAimTurretCmd;
    IntakeLineCmd intakeLineCmd;
    StopIntakeLineCmd stopIntakeLineCmd;
    ResetImuCmd resetImuCmd;

    boolean sortMode;

    /* ------------------- Gamepad Declaration ------------------- */
    private GamepadEx m_driveOp;
    TriggerReader rightTriggerReader;

    /* ------------------- Variable Declarations ------------------- */
    public int pipelineNum = 0;

    /* ---------- Elapsed Time ---------- */
    ElapsedTime time;

    /* --------- Subsystems --------- */
    Subsystems subsystems;

    @Override
    public void initialize() {
        subsystems = new Subsystems(hardwareMap, pipelineNum);

        /* -------------- Gamepad -------------- */
        //Create the gamepad controller
        m_driveOp = new GamepadEx(gamepad1);


        /* -------------- Elapsed time ---------------- */
        time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        /* -------------- COMMANDS -------------- */

        if(pipelineNum == Constants.BLUE_PIPELINE){
            mecDriveCmd = new DriveCommand(subsystems.mecDriveSubsystem, m_driveOp::getLeftX, m_driveOp::getLeftY, m_driveOp::getRightX, subsystems.mecDriveSubsystem::getRobotBlueDriveHeading);
            resetImuCmd = new ResetImuCmd(subsystems.mecDriveSubsystem, -90.0);
        } else if (pipelineNum == Constants.RED_PIPELINE){
            mecDriveCmd = new DriveCommand(subsystems.mecDriveSubsystem, m_driveOp::getLeftX, m_driveOp::getLeftY, m_driveOp::getRightX, subsystems.mecDriveSubsystem::getRobotRedDriveHeading);
            resetImuCmd = new ResetImuCmd(subsystems.mecDriveSubsystem, 90.0);
        } else {
            mecDriveCmd = new DriveCommand(subsystems.mecDriveSubsystem, m_driveOp::getLeftX, m_driveOp::getLeftY, m_driveOp::getRightX, subsystems.mecDriveSubsystem::getRobotHeadingRadians);
            resetImuCmd = new ResetImuCmd(subsystems.mecDriveSubsystem, 0.0);
        }

        runIntakeSortCmd = new IntakeSortCmd(subsystems.intakeSubsystem, subsystems.spindexerSubsystem, subsystems.kickSubsystem);
        stopIntakeCmd = new StopIntakeCmd(subsystems.intakeSubsystem);
        uptakeCmd = new UptakeCmd(subsystems.kickSubsystem);
        neutralAimTurretCmd = new AimTurretCmd(subsystems.aimSubsystem, 90.0);
        intakeLineCmd = new IntakeLineCmd(subsystems.shooterFeedSubsystem, subsystems.intakeSubsystem, subsystems.spindexerSubsystem, subsystems.kickSubsystem);
        stopIntakeLineCmd = new StopIntakeLineCmd(subsystems.shooterFeedSubsystem, subsystems.intakeSubsystem, subsystems.spindexerSubsystem, subsystems.kickSubsystem);

        sortMode = false;



        /* -------------- Button Bindings -------------- */

        rightTriggerReader = new TriggerReader(
                m_driveOp, GamepadKeys.Trigger.RIGHT_TRIGGER
        );


        m_driveOp.getGamepadButton(GamepadKeys.Button.A)
                .whenHeld(new ParallelCommandGroup( new UptakeCmd(subsystems.kickSubsystem),
                        new TransferCmd(subsystems.shooterFeedSubsystem)))
                .whenReleased( new ParallelCommandGroup(new StopUptakeCmd(subsystems.kickSubsystem),
                        new StopTransferCmd(subsystems.shooterFeedSubsystem)));

        m_driveOp.getGamepadButton(GamepadKeys.Button.X)
                .whenHeld(new AimTurretCmd(subsystems.aimSubsystem, Constants.NEUTRAL_TURRET));

        m_driveOp.getGamepadButton(GamepadKeys.Button.B);

        m_driveOp.getGamepadButton(GamepadKeys.Button.Y);


        m_driveOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new MoveTurretNegative(subsystems.aimSubsystem));

        m_driveOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new MoveTurretPositive(subsystems.aimSubsystem));

        m_driveOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new MoveHoodNegative(subsystems.aimSubsystem));

        m_driveOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new MoveHoodPositive(subsystems.aimSubsystem));

        m_driveOp.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed( new AimTurretCmd(subsystems.aimSubsystem, 180.0)
                        .andThen(new ResetPoseCmd(subsystems.mecDriveSubsystem, subsystems.sensorSubsystem, subsystems.sensorSubsystem.getPipeline())));

        m_driveOp.getGamepadButton(GamepadKeys.Button.START)
                .whenHeld(resetImuCmd);

        /* -------------- Driving Command Loop -------------- */
        //this will make the drive command always run
        register(subsystems.mecDriveSubsystem);  //good practice to register the default subsystem

        //Set Field Centric as default drive Command
        subsystems.mecDriveSubsystem.setDefaultCommand(mecDriveCmd); //command that runs automatically whenever a subsystem is not being used by another command


        /* -------------- Update Telemetry -------------- */
        // update telemetry every loop

        schedule(new RunCommand(telemetry::update));

    }

    public void runOpMode(){
        CommandScheduler.getInstance().reset();
        initialize();
        waitForStart();

        time.reset();
        new ResetCmd(subsystems.shooterSubsystem, subsystems.shooterFeedSubsystem, subsystems.spindexerSubsystem, subsystems.aimSubsystem, subsystems.kickSubsystem, subsystems.intakeSubsystem).schedule();
        new AimTurretCmd(subsystems.aimSubsystem, Constants.NEUTRAL_TURRET).schedule();
        sleep(250);

        subsystems.aimSubsystem.aimHood(0.0);
        subsystems.kickSubsystem.rotateKickerUp();
        subsystems.spindexerSubsystem.runToSlotZero();

        while (opModeIsActive()){
            //Scheduler must be loop called for everything else to run
            CommandScheduler.getInstance().run();
            rightTriggerReader.readValue();

            telemetry.addData("Top beam break is broken: ", subsystems.shooterFeedSubsystem.topBroken());
            telemetry.addData("Ball in Bottom Beam: ", subsystems.intakeSubsystem.ballInBeam());

            telemetry.addData("Turret Degrees: ", subsystems.aimSubsystem.getTurretDegrees());
            telemetry.addData("Turn to with PP", subsystems.mecDriveSubsystem.getPinpointTurretAngle(pipelineNum));
            telemetry.addData("Kicker Rotate", subsystems.kickSubsystem.getKickerRotate());


            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), subsystems.mecDriveSubsystem.getBotPose());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            if(isStopRequested()){
                SavedVars.reset();
            }
        }
    }
}
