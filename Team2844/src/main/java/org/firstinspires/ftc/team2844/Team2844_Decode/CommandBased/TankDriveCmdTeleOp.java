package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
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

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.DriveCmdArcade;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.DriveCmdTank;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.IntakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.LimelightSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.ShooterSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.SpindexerSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.TankDriveSubsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

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
    IntakeCmd stopIntakeCmd;
    IntakeCmd runIntakeCmd;

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

        //Create motor group with Front as the leader
        leftMotorGroup = new MotorGroup(this.frontLeft,this.backLeft);
        rightMotorGroup = new MotorGroup(this.frontRight,this.backRight);

        // ----- Shooter Motors ----- //
        shooterLeft = new MotorEx(hardwareMap, "shooterLeft");
        shooterRight = new MotorEx(hardwareMap, "shooterRight");
        shooterLeft.setInverted(true);

        shooterPair = new MotorExGroup(new ArrayList<>(Arrays.asList(this.shooterLeft, this.shooterRight)));
        shooterPair.setRunMode(Motor.RunMode.VelocityControl);

        // ----- Aim Servos ----- //
        turretAim = hardwareMap.get(Servo.class, "turretAim");
        hoodAim = hardwareMap.get(Servo.class, "hoodAim");

        // ----- Intake ----- //
        intakeMotor = new Motor(hardwareMap, "intakeMotor");
        intakeMotor.setInverted(true);

        // ----- Kicker Servos ----- //
        kickerRotate = hardwareMap.get(Servo.class, "kickerRotate");
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
        driveCmdArcade = new DriveCmdArcade(tankDriveSubsystem, m_driveOp::getLeftY, m_driveOp::getLeftX);
        runIntakeCmd = new IntakeCmd(intakeSubsystem, 0.75);
        stopIntakeCmd = new IntakeCmd(intakeSubsystem, 0.0);






        /* -------------- Button Bindings -------------- */

        rightTriggerReader = new TriggerReader(
                m_driveOp, GamepadKeys.Trigger.RIGHT_TRIGGER
        );

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


        while (opModeIsActive()){
            //Scheduler must be loop called for everything else to run
            CommandScheduler.getInstance().run();


            //Right trigger press checking, if true, runs intake, else stops intake (may cause issues later if constantly scheduling stop...)
            if ( rightTriggerReader.isDown() ) {
                runIntakeCmd.schedule();
                telemetry.addData("Right Trigger is down: ", true);
            } else {
                stopIntakeCmd.schedule();
                telemetry.addData("Right Trigger is down: ", false);
            }


            rightTriggerReader.readValue();
        }
    }
}
