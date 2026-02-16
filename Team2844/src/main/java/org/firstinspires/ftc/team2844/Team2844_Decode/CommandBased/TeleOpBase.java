package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.AimTurretCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.MoveHoodNegative;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.MoveHoodPositive;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.MoveTurretNegative;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.MoveTurretPositive;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.DriveCommands.DriveCommand;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.DriveCommands.ResetImuCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.IntakeLineCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.IntakeSortCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.StopIntakeLineCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.StopIntakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.NeutralShooterCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.SmartLineShooterCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.SmartSortShootCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.StopTransferCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.TransferCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.SlotCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.StopUptakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.UptakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.SavedVars;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Subsystems;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.DriveSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.SensorSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterFeedSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SpindexerSubsystem;

import java.util.ArrayList;
import java.util.Arrays;

@Disabled
public class TeleOpBase extends CommandOpMode {
    /* ------------------- Command Declarations ------------------- */

    //private DriveCmdTank driveCmdTank;
    //private DriveCmdArcade driveCmdArcade;
    private DriveCommand mecDriveCmd;
    StopIntakeCmd stopIntakeCmd;
    IntakeSortCmd runIntakeSortCmd;
    UptakeCmd uptakeCmd;
    AimTurretCmd neutralAimTurretCmd;
    IntakeLineCmd intakeLineCmd;
    StopIntakeLineCmd stopIntakeLineCmd;

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

            //Create a new drive command and pass in the drive subsystem and the gamepad control values
        //driveCmdTank = new DriveCmdTank(tankDriveSubsystem, m_driveOp::getLeftY, m_driveOp::getRightY);
            //Arcade Drive controls
        //driveCmdArcade = new DriveCmdArcade(tankDriveSubsystem, m_driveOp::getLeftY, m_driveOp::getRightX);
        mecDriveCmd = new DriveCommand(subsystems.mecDriveSubsystem, m_driveOp::getLeftX, m_driveOp::getLeftY, m_driveOp::getRightX, subsystems.mecDriveSubsystem.getRobotHeading());

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

        m_driveOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new SmartLineShooterCmd(subsystems.shooterSubsystem, subsystems.shooterFeedSubsystem, subsystems.sensorSubsystem, subsystems.aimSubsystem, subsystems.kickSubsystem, subsystems.intakeSubsystem, subsystems.mecDriveSubsystem))
                .whenReleased(new NeutralShooterCmd(subsystems.shooterSubsystem, subsystems.shooterFeedSubsystem, subsystems.aimSubsystem, subsystems.kickSubsystem, subsystems.intakeSubsystem));

        m_driveOp.getGamepadButton(GamepadKeys.Button.A)
                .whenHeld(new ParallelCommandGroup( new UptakeCmd(subsystems.kickSubsystem),
                        new TransferCmd(subsystems.shooterFeedSubsystem)))
                .whenReleased( new ParallelCommandGroup(new StopUptakeCmd(subsystems.kickSubsystem),
                        new StopTransferCmd(subsystems.shooterFeedSubsystem)));

        m_driveOp.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new SlotCmd(subsystems.spindexerSubsystem, subsystems.kickSubsystem, 0));

        m_driveOp.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new SlotCmd(subsystems.spindexerSubsystem, subsystems.kickSubsystem, 1));

        m_driveOp.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new SlotCmd(subsystems.spindexerSubsystem, subsystems.kickSubsystem, 2));


        m_driveOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenHeld(new MoveTurretNegative(subsystems.aimSubsystem));

        m_driveOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenHeld(new MoveTurretPositive(subsystems.aimSubsystem));

        m_driveOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenHeld(new MoveHoodNegative(subsystems.aimSubsystem));

        m_driveOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenHeld(new MoveHoodPositive(subsystems.aimSubsystem));

        m_driveOp.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(new ResetImuCmd(subsystems.mecDriveSubsystem));



        /* -------------- Driving Command Loop -------------- */
            //this will make the drive command always run
        register(subsystems.mecDriveSubsystem);  //good practice to register the default subsystem

            //Set Tank Drive as default drive Command
        //tankDriveSubsystem.setDefaultCommand(driveCmdTank); //command that runs automatically whenever a subsystem is not being used by another command

            //Set Arcade Drive as default drive Command
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
        new NeutralShooterCmd(subsystems.shooterSubsystem, subsystems.shooterFeedSubsystem, subsystems.aimSubsystem, subsystems.kickSubsystem, subsystems.intakeSubsystem).schedule();
        sleep(250);

        subsystems.aimSubsystem.aimHood(0.0);
        subsystems.spindexerSubsystem.runToSlotZero();

        while (opModeIsActive()){
            //Scheduler must be loop called for everything else to run
            CommandScheduler.getInstance().run();
            rightTriggerReader.readValue();

            if(m_driveOp.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) && subsystems.spindexerSubsystem.empty()){
                sortMode = !sortMode;
            }

            //Right trigger press checking, if true, runs intake, else stops intake (may cause issues later if constantly scheduling stop...)
            /*if ( rightTriggerReader.isDown() && !sortMode) {
                new IntakeLineCmd(subsystems.shooterFeedSubsystem, subsystems.intakeSubsystem, subsystems.spindexerSubsystem, subsystems.kickSubsystem).schedule();
            } else if (rightTriggerReader.wasJustReleased() && !sortMode) {
                new StopIntakeLineCmd(subsystems.shooterFeedSubsystem, subsystems.intakeSubsystem, subsystems.spindexerSubsystem, subsystems.kickSubsystem).schedule();
            } else if (rightTriggerReader.isDown() && sortMode){
                new IntakeSortCmd(subsystems.intakeSubsystem, subsystems.spindexerSubsystem, subsystems.kickSubsystem).schedule();
            } else if (rightTriggerReader.wasJustReleased() && sortMode){
                new StopIntakeCmd(subsystems.intakeSubsystem);
            }

            if(!sortMode){
                m_driveOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                        .whileHeld(new SmartLineShooterCmd(subsystems.shooterSubsystem, subsystems.shooterFeedSubsystem, subsystems.sensorSubsystem, subsystems.aimSubsystem, subsystems.kickSubsystem, subsystems.intakeSubsystem))
                        .whenReleased(new NeutralShooterCmd(subsystems.shooterSubsystem, subsystems.shooterFeedSubsystem, subsystems.aimSubsystem, subsystems.kickSubsystem, subsystems.intakeSubsystem));
            } else {
                m_driveOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                        .whileHeld(new SmartSortShootCmd(subsystems.shooterSubsystem, subsystems.shooterFeedSubsystem, subsystems.sensorSubsystem, subsystems.aimSubsystem, subsystems.spindexerSubsystem, subsystems.kickSubsystem))
                        .whenReleased(new NeutralShooterCmd(subsystems.shooterSubsystem, subsystems.shooterFeedSubsystem, subsystems.aimSubsystem, subsystems.kickSubsystem, subsystems.intakeSubsystem));
            }*/

            if ( rightTriggerReader.isDown()) {
                new IntakeLineCmd(subsystems.shooterFeedSubsystem, subsystems.intakeSubsystem, subsystems.spindexerSubsystem, subsystems.kickSubsystem).schedule();
            } else if (rightTriggerReader.wasJustReleased()) {
                new StopIntakeLineCmd(subsystems.shooterFeedSubsystem, subsystems.intakeSubsystem, subsystems.spindexerSubsystem, subsystems.kickSubsystem).schedule();
            }

            subsystems.sensorSubsystem.updateOrientation(subsystems.mecDriveSubsystem.getRobotHeading());


            telemetry.addData("Limelight Tx: ", subsystems.sensorSubsystem.getTx());
            telemetry.addData("Average Distance from MetaTag: ", subsystems.sensorSubsystem.getDis());
            telemetry.addData("Limelight Bot X: ", subsystems.sensorSubsystem.getBotXLL());
            telemetry.addData("Limelight Bot Y: ", subsystems.sensorSubsystem.getBotYLL());

            telemetry.addData("Imu Degrees: ", subsystems.mecDriveSubsystem.getRobotHeading());
            telemetry.addData("Pinpoint Bot X: ", subsystems.mecDriveSubsystem.getBotX());
            telemetry.addData("Pinpoint Bot Y: ", subsystems.mecDriveSubsystem.getBotY());
            telemetry.addData("Turret Degrees: ", subsystems.aimSubsystem.getTurretDegrees());

            telemetry.addData("Raw Axon Voltage: ", subsystems.aimSubsystem.getVoltage());
            telemetry.addData("Axon degrees: ", subsystems.aimSubsystem.getAxonValue());
            telemetry.addData("Slot: ", subsystems.spindexerSubsystem.getSlot());
            telemetry.addData("Spindexer Pos: ", subsystems.spindexerSubsystem.getPosition());

            telemetry.addData("Top beam break is broken: ", subsystems.shooterFeedSubsystem.topBroken());
            telemetry.addData("Ball in Beam: ", subsystems.intakeSubsystem.ballInBeam());

            telemetry.addData("Ball in Bay One: ", subsystems.spindexerSubsystem.ballInBayOne());
            telemetry.addData("Ball in Bay Two: ", subsystems.spindexerSubsystem.ballInBayTwo());
            telemetry.addData("Ball in Bay Three: ", subsystems.spindexerSubsystem.ballInBayThree());
            telemetry.addData("Bay one alpha", subsystems.spindexerSubsystem.bayOneAlpha());
            telemetry.addData("Bay two alpha", subsystems.spindexerSubsystem.bayTwoAlpha());
            telemetry.addData("Bay Three alpha", subsystems.spindexerSubsystem.bayThreeAlpha());
        }

        SavedVars.reset();
    }

    /*public void telemetry(){
        telemetry.addData("The rotating kicker value: ", subsystems.kickSubsystem.getKickerRotate());
        telemetry.addData("The Value of the Turret: ", subsystems.aimSubsystem.getAxonValue());

        telemetry.addData("Ball in Bay One: ", subsystems.spindexerSubsystem.ballInBayOne());
        telemetry.addData("Ball in Bay Two: ", subsystems.spindexerSubsystem.ballInBayTwo());

        telemetry.addData("Ball in Bay Three: ", subsystems.spindexerSubsystem.ballInBayThree());
        telemetry.addData("Color in Bay One: ",  subsystems.spindexerSubsystem.bayOneColor());
        telemetry.addData("Color in Bay Two: ",  subsystems.spindexerSubsystem.bayTwoColor());
        telemetry.addData("Color in Bay Three: ",subsystems.spindexerSubsystem.bayThreeColor());

        telemetry.addData("Bay Number",          subsystems.spindexerSubsystem.getSlot());

        //Bay One
        telemetry.addData("Bay One Blue Values: ", subsystems.spindexerSubsystem.bayOneBlue()[0] + ", " +subsystems.spindexerSubsystem.bayOneBlue()[1]);
        telemetry.addData("Bay One Red Values: ",  subsystems.spindexerSubsystem.bayOneRed()[0] + ", " + subsystems.spindexerSubsystem.bayOneRed()[1]);
        telemetry.addData("Bay One Green Values: ",subsystems.spindexerSubsystem.bayOneGreen()[0] + ", "+subsystems.spindexerSubsystem.bayOneGreen()[1]);

        //Bay Two
        telemetry.addData("Bay Two Blue Values: ", subsystems.spindexerSubsystem.bayTwoBlue()[0] + ", " +subsystems.spindexerSubsystem.bayTwoBlue()[1]);
        telemetry.addData("Bay Two Red Values: ",  subsystems.spindexerSubsystem.bayTwoRed()[0] + ", " + subsystems.spindexerSubsystem.bayTwoRed()[1]);
        telemetry.addData("Bay Two Green Values: ",subsystems.spindexerSubsystem.bayTwoGreen()[0] + ", "+subsystems.spindexerSubsystem.bayTwoGreen()[1]);

        //Bay Three
        telemetry.addData("Bay Three Blue Values: ",subsystems.spindexerSubsystem.bayThreeBlue()[0] + ", " +subsystems.spindexerSubsystem.bayThreeBlue()[1]);
        telemetry.addData("Bay Three Red Values: ",subsystems.spindexerSubsystem.bayThreeRed()[0] + ", " +  subsystems.spindexerSubsystem.bayThreeRed()[1]);
        telemetry.addData("Bay Three Green Values: ",subsystems.spindexerSubsystem.bayThreeGreen()[0] + ", "+subsystems.spindexerSubsystem.bayThreeGreen()[1]);
    }*/
}
