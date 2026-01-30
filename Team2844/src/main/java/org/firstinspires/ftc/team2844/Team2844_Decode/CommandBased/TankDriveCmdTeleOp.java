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
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.AimHoodCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.AimTurretCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.MoveHoodNegative;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.MoveHoodPositive;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.MoveTurretNegative;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.MoveTurretPositive;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.DriveCommands.DriveCmdArcade;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.DriveCommands.DriveCmdTank;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.IntakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.ShootCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCommands.StopIntakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.StopShootCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.StopTransferCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.ShootingCommands.TransferCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.BayOneCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.BayThreeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.BayTwoCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.StopUptakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.UptakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.LimelightSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SpindexerSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.TankDriveSubsystem;

import java.util.ArrayList;
import java.util.Arrays;

@Disabled
public class TankDriveCmdTeleOp extends CommandOpMode {
    /* ------------------- Motor Declarations -------------------*/
    /**Drive motor parameter pass-ins*/
    private Motor frontLeft, frontRight, backLeft, backRight;
    /**Drive motor group parameter pass-ins*/
    private MotorGroup leftMotorGroup, rightMotorGroup;

    /**Motors for shooter subsystem pass-ins (one the motors in the pair of shooter motors)*/
    private MotorEx shooterLeft, shooterRight;
    /**Motor group of the shooters, the left shooter should be inverted*/
    private MotorExGroup shooterPair;

    /**Intake Motor*/
    private Motor intakeMotor;

    /* ------------------- Servo Declarations ------------------- */
    /**Aiming servo object parameter pass-ins*/
    private Servo hoodAim, turretAim;

    /**Servo objects for passing into kicksubsystem*/
    private Servo kickerRotate;
    /**CRServo objects for passing into kicksubsystem*/
    private CRServo kickerSpin, sFeed;

    /**Servo objects for passing into spindexerSubsystem*/
    private Servo spindexer;

    /**Servo objects for passing into ShooterSubsystem*/
    private CRServo tFeed;

    /* ------------------- Limelight ------------------- */
    private Limelight3A limelight;

    /* ------------------- Subsystem Declarations ------------------- */
    /**Drive Subsystem*/
    private TankDriveSubsystem tankDriveSubsystem;
    /**Intake Subsystem*/
    private IntakeSubsystem intakeSubsystem;
    /**Aim Subsystem*/
    private AimSubsystem aimSubsystem;
    /**Shooter Subsystem*/
    private ShooterSubsystem shooterSubsystem;
    /**Kick Subsystem*/
    private KickSubsystem kickSubsystem;
    /**Spindexer Subsystem*/
    private SpindexerSubsystem spindexerSubsystem;
    /**Limelight Subsystem*/
    private LimelightSubsystem limelightSubsystem;
    /* ------------------- Command Declarations ------------------- */

    private DriveCmdTank driveCmdTank;
    private DriveCmdArcade driveCmdArcade;
    StopIntakeCmd stopIntakeCmd;
    IntakeCmd runIntakeCmd;
    UptakeCmd uptakeCmd;

    /* ------------------- Gamepad Declaration ------------------- */
    private GamepadEx m_driveOp;
    TriggerReader rightTriggerReader;

    /* ------------------- Variable Declarations ------------------- */
    public int pipelineNum = 0;

    @Override
    public void initialize() {
        /* -------------- Hardware Maps -------------- */

        // ----- Drive Motors ----- //
        //Map HW motors to variables
        frontLeft = new Motor(hardwareMap, "frontLeft");
        frontRight = new Motor(hardwareMap, "frontRight");
        backLeft = new Motor(hardwareMap, "backLeft");
        backRight = new Motor(hardwareMap, "backRight");

        backLeft.setInverted(false);
        backRight.setInverted(false);

        //Create motor group with Front as the leader
        leftMotorGroup = new MotorGroup(this.frontLeft,this.backLeft);
        rightMotorGroup = new MotorGroup(this.frontRight,this.backRight);

//        rightMotorGroup.setInverted(true);

        // ----- Shooter Motors ----- //
        shooterLeft = new MotorEx(hardwareMap, "shooterLeft");
        shooterRight = new MotorEx(hardwareMap, "shooterRight");
        shooterLeft.setInverted(true);

        shooterPair = new MotorExGroup(new ArrayList<>(Arrays.asList(this.shooterLeft, this.shooterRight)));
        shooterPair.setRunMode(Motor.RunMode.RawPower);
        shooterPair.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        // ----- Shooter Servos ----- //
        tFeed = hardwareMap.get(CRServo.class, "tFeed");

        // ----- Aim Servos ----- //
        turretAim = hardwareMap.get(Servo.class, "turretAim");
        hoodAim = hardwareMap.get(Servo.class, "hoodAim");

        // ----- Intake ----- //
        intakeMotor = new Motor(hardwareMap, "intakeMotor");
        intakeMotor.setInverted(true);

        // ----- Kicker Servos ----- //
        kickerRotate = hardwareMap.get(Servo.class, "kickerRotate");
        kickerRotate.setPosition(0.0);
        sFeed = hardwareMap.get(CRServo.class, "sFeed");
        kickerSpin = hardwareMap.get(CRServo.class, "kickerSpin");


        // ----- Spindexer Servo ----- //
        spindexer = hardwareMap.get(Servo.class, "spindexer");

        // ----- Limelight ----- //
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        /* -------------- Gamepad -------------- */
        //Create the gamepad controller
        m_driveOp = new GamepadEx(gamepad1);


        /* -------------- Subsystems -------------- */
        /* 1. drive
         * 2. intake
         * 3. Kick
         * 4. Spindexer
         * 5. Limelight
         * 6. Shooter
         * 7. Aim*/

        //1. drive subsystem
        tankDriveSubsystem = new TankDriveSubsystem(leftMotorGroup,rightMotorGroup);
        //2. Intake subsystem
        intakeSubsystem = new IntakeSubsystem(intakeMotor);
        //3. Kick subsystem
        kickSubsystem = new KickSubsystem(kickerRotate, kickerSpin, sFeed);
        //4. Spindexer subsystem
        spindexerSubsystem = new SpindexerSubsystem(this, spindexer);
        //5. Limelight subsystem
        limelightSubsystem = new LimelightSubsystem(limelight, pipelineNum);
        //6. Shooter subsystem
        shooterSubsystem = new ShooterSubsystem(shooterPair, tFeed);
        //7. Aim subsystem
        aimSubsystem = new AimSubsystem(hoodAim, turretAim);


        /* -------------- COMMANDS -------------- */

            //Create a new drive command and pass in the drive subsystem and the gamepad control values
        //driveCmdTank = new DriveCmdTank(tankDriveSubsystem, m_driveOp::getLeftY, m_driveOp::getRightY);
            //Arcade Drive controls
        driveCmdArcade = new DriveCmdArcade(tankDriveSubsystem, m_driveOp::getLeftY, m_driveOp::getRightX);
        runIntakeCmd = new IntakeCmd(intakeSubsystem, spindexerSubsystem);
        stopIntakeCmd = new StopIntakeCmd(intakeSubsystem);
        uptakeCmd = new UptakeCmd(kickSubsystem);






        /* -------------- Button Bindings -------------- */

        rightTriggerReader = new TriggerReader(
                m_driveOp, GamepadKeys.Trigger.RIGHT_TRIGGER
        );

        m_driveOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenHeld(new ShootCmd(shooterSubsystem))
                        .whenReleased(new StopShootCmd(shooterSubsystem));

        m_driveOp.getGamepadButton(GamepadKeys.Button.A)
                .whenHeld(new ParallelCommandGroup( new UptakeCmd(kickSubsystem),
                        new TransferCmd(shooterSubsystem)))
                .whenReleased( new ParallelCommandGroup(new StopUptakeCmd(kickSubsystem),
                        new StopTransferCmd(shooterSubsystem)));

        m_driveOp.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new BayOneCmd(spindexerSubsystem));

        m_driveOp.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new BayTwoCmd(spindexerSubsystem));

        m_driveOp.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new BayThreeCmd(spindexerSubsystem));


        m_driveOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenHeld(new MoveTurretNegative(aimSubsystem));

        m_driveOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenHeld(new MoveTurretPositive(aimSubsystem));

        m_driveOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenHeld(new MoveHoodNegative(aimSubsystem));

        m_driveOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenHeld(new MoveHoodPositive(aimSubsystem));



/*
        m_driveOp.getGamepadButton(GamepadKeys.Button.B)
                .whenHeld(new InstantCommand(intakeSubsystem::reverse, intakeSubsystem))
                .whenReleased(new InstantCommand(intakeSubsystem::stop, intakeSubsystem));
*/


        /* -------------- Driving Command Loop -------------- */
            //this will make the drive command always run
        register(tankDriveSubsystem);  //good practice to register the default subsystem

            //Set Tank Drive as default drive Command
        //tankDriveSubsystem.setDefaultCommand(driveCmdTank); //command that runs automatically whenever a subsystem is not being used by another command

            //Set Arcade Drive as default drive Command
        tankDriveSubsystem.setDefaultCommand(driveCmdArcade); //command that runs automatically whenever a subsystem is not being used by another command


        /* -------------- Update Telemetry -------------- */
            // update telemetry every loop

        schedule(new RunCommand(telemetry::update));

    }

    public void runOpMode(){
        CommandScheduler.getInstance().reset();
        initialize();
        waitForStart();

        aimSubsystem.aimTurret(0.5);
        aimSubsystem.aimHood(0.0);

        while (opModeIsActive()){
            //Scheduler must be loop called for everything else to run
            CommandScheduler.getInstance().run();


            //Right trigger press checking, if true, runs intake, else stops intake (may cause issues later if constantly scheduling stop...)
            if ( rightTriggerReader.isDown() ) {
                runIntakeCmd.schedule();
            } else {
                stopIntakeCmd.schedule();
            }

            telemetry();
            rightTriggerReader.readValue();
        }
    }

    public void telemetry(){
        telemetry.addData("The rotating kicker value: ", kickSubsystem.getKickerRotate());
        telemetry.addData("The Value of the Turret: ", aimSubsystem.getTurretValue());

        telemetry.addData("Ball in Bay One: ", spindexerSubsystem.ballInBayOne());
        telemetry.addData("Ball in Bay Two: ", spindexerSubsystem.ballInBayTwo());
        telemetry.addData("Ball in Bay Three: ", spindexerSubsystem.ballInBayThree());

        telemetry.addData("Color in Bay One: ", spindexerSubsystem.bayOneColor());
        telemetry.addData("Color in Bay Two: ", spindexerSubsystem.bayTwoColor());
        telemetry.addData("Color in Bay Three: ", spindexerSubsystem.bayThreeColor());

        telemetry.addData("Bay Number", spindexerSubsystem.getBay());

        //Bay One
        telemetry.addData("Bay One Blue Values: ", spindexerSubsystem.bayOneBlue()[0] + ", " +spindexerSubsystem.bayOneBlue()[1]);
        telemetry.addData("Bay One Red Values: ", spindexerSubsystem.bayOneRed()[0] + ", " +spindexerSubsystem.bayOneRed()[1]);
        telemetry.addData("Bay One Green Values: ", spindexerSubsystem.bayOneGreen()[0] + ", " +spindexerSubsystem.bayOneGreen()[1]);

        //Bay Two
        telemetry.addData("Bay Two Blue Values: ", spindexerSubsystem.bayTwoBlue()[0] + ", " +spindexerSubsystem.bayTwoBlue()[1]);
        telemetry.addData("Bay Two Red Values: ", spindexerSubsystem.bayTwoRed()[0] + ", " +spindexerSubsystem.bayTwoRed()[1]);
        telemetry.addData("Bay Two Green Values: ", spindexerSubsystem.bayTwoGreen()[0] + ", " +spindexerSubsystem.bayTwoGreen()[1]);

        //Bay Three
        telemetry.addData("Bay Three Blue Values: ", spindexerSubsystem.bayThreeBlue()[0] + ", " +spindexerSubsystem.bayThreeBlue()[1]);
        telemetry.addData("Bay Three Red Values: ", spindexerSubsystem.bayThreeRed()[0] + ", " +spindexerSubsystem.bayThreeRed()[1]);
        telemetry.addData("Bay Three Green Values: ", spindexerSubsystem.bayThreeGreen()[0] + ", " +spindexerSubsystem.bayThreeGreen()[1]);
    }
}
