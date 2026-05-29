package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autonomous.Roadrunner.Drawing;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.AimTurretCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.DefaultAimCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.HoodCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.MoveHoodNegative;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.MoveHoodPositive;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.MoveTurretNegative;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.MoveTurretPositive;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.DriveCommands.DriveCommand;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.DriveCommands.ResetImuCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.DriveCommands.ResetPoseCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.FullExtakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.IntakeLineCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.StopIntakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.LightCommands.TimerLightsCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.DefaultVelocityShootCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.ResetCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.SmartLineShooterCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.TransferCommands.FullTransferCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.TransferCommands.StopTransferCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.StopUptakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Constants;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.SavedVars;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Subsystems;

import java.util.function.BooleanSupplier;

@Disabled
public class TeleOpBase extends CommandOpMode {
    /* ------------------- Command Declarations ------------------- */

    //private DriveCmdTank driveCmdTank;
    //private DriveCmdArcade driveCmdArcade;
    private DriveCommand mecDriveCmd;

    IntakeLineCmd intakeLineCmd;
    FullExtakeCmd extakeCmd;
    ResetImuCmd resetImuCmd;
    TimerLightsCmd timerLights;

    boolean sortMode;
    boolean intake = false;

    boolean manualAimPrim = false;
    BooleanSupplier manualAim = ()->manualAimPrim;

    /* ------------------- Gamepad Declaration ------------------- */
    private GamepadEx m_driveOp;
    TriggerReader rightTriggerReader;
    TriggerReader leftTriggerReader;

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

        intakeLineCmd = new IntakeLineCmd(subsystems.shooterFeedSubsystem, subsystems.intakeSubsystem, subsystems.spindexerSubsystem, subsystems.kickSubsystem);
        extakeCmd = new FullExtakeCmd(subsystems.intakeSubsystem, subsystems.shooterFeedSubsystem, subsystems.kickSubsystem);
        timerLights = new TimerLightsCmd(subsystems.lightSubsystem, Constants.BOTL_INDEX);

        sortMode = false;



        /* -------------- Button Bindings -------------- */

        rightTriggerReader = new TriggerReader(
                m_driveOp, GamepadKeys.Trigger.RIGHT_TRIGGER
        );
        leftTriggerReader = new TriggerReader(
                m_driveOp, GamepadKeys.Trigger.LEFT_TRIGGER
        );

        m_driveOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new SmartLineShooterCmd(subsystems, manualAim))
                .whenReleased(new ResetCmd(subsystems));

        m_driveOp.getGamepadButton(GamepadKeys.Button.A)
                .whenHeld(new FullTransferCmd(subsystems.shooterFeedSubsystem, subsystems.intakeSubsystem, subsystems.kickSubsystem))
                .whenReleased( new ParallelCommandGroup(new StopUptakeCmd(subsystems.kickSubsystem),
                        new StopTransferCmd(subsystems.shooterFeedSubsystem),
                        new StopIntakeCmd(subsystems.intakeSubsystem)));

        m_driveOp.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(()-> manualAimPrim = !manualAimPrim))
                .whenPressed(new AimTurretCmd(subsystems.aimSubsystem, Constants.NEUTRAL_TURRET));

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
                .whileHeld(resetImuCmd);


        //Default Commands
        register(subsystems.aimSubsystem, subsystems.shooterSubsystem);
        subsystems.aimSubsystem.setDefaultCommand(new DefaultAimCmd(subsystems.aimSubsystem, subsystems.mecDriveSubsystem, subsystems.sensorSubsystem, manualAim));
        subsystems.aimSubsystem.setDefaultCommand(new HoodCmd(subsystems.aimSubsystem, () -> subsystems.mecDriveSubsystem.hoodLinReg(pipelineNum)));
        subsystems.shooterSubsystem.setDefaultCommand(new DefaultVelocityShootCmd(subsystems));

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
        new ResetCmd(subsystems).schedule();
        new AimTurretCmd(subsystems.aimSubsystem, Constants.NEUTRAL_TURRET).schedule();
        sleep(250);

        subsystems.aimSubsystem.aimHood(0.0);
        subsystems.spindexerSubsystem.runToSlotZero();

        while (opModeIsActive()){
            //Scheduler must be loop called for everything else to run
            CommandScheduler.getInstance().run();
            rightTriggerReader.readValue();
            leftTriggerReader.readValue();

            if(!timerLights.isScheduled()){
                timerLights.schedule();
            }

            if ( rightTriggerReader.wasJustPressed()) {
                intake = !intake;
            } else if(leftTriggerReader.wasJustPressed() && !extakeCmd.isScheduled()){
                extakeCmd.schedule();
            } else if(leftTriggerReader.wasJustReleased()){
                extakeCmd.cancel();
            }
//            } else if (rightTriggerReader.wasJustReleased()) {
//                intakeLineCmd.interruptOn(() ->true);
//                new StopIntakeLineCmd(subsystems.shooterFeedSubsystem, subsystems.intakeSubsystem, subsystems.spindexerSubsystem, subsystems.kickSubsystem).schedule();
//            }

            if(intake  && !intakeLineCmd.isScheduled()){
                intakeLineCmd.schedule(true);
            }else {
                intakeLineCmd.cancel();
            }



            double cameraHeading = subsystems.mecDriveSubsystem.getRobotHeading() + (subsystems.aimSubsystem.getTurretDegrees()-180);
            while(cameraHeading > 180) cameraHeading -= 360;
            while (cameraHeading <= -180) cameraHeading += 360;
            subsystems.sensorSubsystem.updateOrientation(cameraHeading);

            subsystems.shooterSubsystem.setPIDs(
                    Constants.P_GAIN,
                    Constants.I_GAIN,
                    Constants.D_GAIN
            );
            subsystems.shooterSubsystem.setFeedForward(
                    Constants.VEL_KS,
                    Constants.VEL_KV * (12.0/hardwareMap.voltageSensor.iterator().next().getVoltage())
            );


            //stelemetry.addData("Camera Heading", cameraHeading);
            telemetry.addData("Limelight Tx: ", subsystems.sensorSubsystem.getTx());
            //telemetry.addData("Limelight Bot X: ", subsystems.sensorSubsystem.getBotXLL());
            //telemetry.addData("Limelight Bot Y: ", subsystems.sensorSubsystem.getBotYLL());

            telemetry.addData("Limelight Bot X MT2: ", subsystems.sensorSubsystem.getBotXLLMT2());
            telemetry.addData("Limelight Bot Y MT2: ", subsystems.sensorSubsystem.getBotYLLMT2());

            telemetry.addData("Imu Degrees: ", subsystems.mecDriveSubsystem.getRobotHeading());
            telemetry.addData("Pinpoint Bot X: ", subsystems.mecDriveSubsystem.getBotX());
            telemetry.addData("Pinpoint Bot Y: ", subsystems.mecDriveSubsystem.getBotY());
            telemetry.addData("Turret Degrees: ", subsystems.aimSubsystem.getTurretDegrees());
            telemetry.addData("Turn to with PP", subsystems.mecDriveSubsystem.getPinpointTurretAngle(pipelineNum));

            //telemetry.addData("Top beam break is broken: ", subsystems.shooterFeedSubsystem.topBroken());
            //telemetry.addData("Ball in Bottom Beam: ", subsystems.intakeSubsystem.ballInBeam());

            //telemetry.addData("LL Distance", subsystems.sensorSubsystem.getDis());
            //telemetry.addData("Average Distance", subsystems.sensorSubsystem.avgDis(subsystems.mecDriveSubsystem.pinpointDistance(pipelineNum)));
            telemetry.addData("Pinpoint distance", subsystems.mecDriveSubsystem.pinpointDistance(pipelineNum));
            telemetry.addData("expected Velocity", subsystems.mecDriveSubsystem.velocityLinReg(pipelineNum));
            telemetry.addData("Velocity: ", subsystems.shooterSubsystem.getVelocity());
            //telemetry.addData("Shooter Power", subsystems.shooterSubsystem.getPower());
            //telemetry.addData("In range: ", subsystems.shooterSubsystem.inRange());

            telemetry.addData("Left Front Power: ", subsystems.mecDriveSubsystem.drive.leftFront.getPower());
            telemetry.addData("Left Back Power: ", subsystems.mecDriveSubsystem.drive.leftBack.getPower());
            telemetry.addData("Right Front Power: ", subsystems.mecDriveSubsystem.drive.rightFront.getPower());
            telemetry.addData("Right Back Power: ", subsystems.mecDriveSubsystem.drive.rightBack.getPower());


            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), subsystems.mecDriveSubsystem.getBotPose());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            if(isStopRequested()){
                SavedVars.reset();
            }
        }
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

    //make do this stuff
    //intake off when ball in bay one
    //keep kick down until beam is broken and extra with timeout
}
