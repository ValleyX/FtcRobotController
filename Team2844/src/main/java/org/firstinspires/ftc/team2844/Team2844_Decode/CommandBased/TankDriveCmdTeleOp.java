package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
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
import com.qualcomm.hardware.rev.Rev9AxisImuOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.AimingCommands.FullAimToLLCmd;
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
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.SlotCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.StopUptakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Commands.SpindexingCommands.UptakeCmd;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems.PinpointSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.AimSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.SortingSubsystems.KickSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.ShootingSubsystems.LimelightSubsystem;
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

    /***/
    private Motor tFeed;

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
    /**Pinpoint Subsystem*/
    private PinpointSubsystem pinpointSubsystem;
    /* ------------------- Command Declarations ------------------- */

    private DriveCmdTank driveCmdTank;
    private DriveCmdArcade driveCmdArcade;
    StopIntakeCmd stopIntakeCmd;
    IntakeCmd runIntakeCmd;
    UptakeCmd uptakeCmd;
    FullAimToLLCmd aimCmd;

    /* ------------------- Gamepad Declaration ------------------- */
    private GamepadEx m_driveOp;
    TriggerReader rightTriggerReader;

    /* ------------------- Variable Declarations ------------------- */
    public int pipelineNum = 0;

    /* ------------------- Sensor Declarations ------------------- */
    private IMU turretIMU;
    private Motor turretServoEncoder;
    private GoBildaPinpointDriver pinpoint;
    private DigitalChannel topBreak;
    private DigitalChannel intakeBB;



    @Override
    public void initialize() {
        /* -------------- Hardware Maps -------------- */

        // ----- Drive Motors ----- //
        //Map HW motors to variables
        frontLeft = new Motor(hardwareMap, Constants.CM0);
        frontRight = new Motor(hardwareMap, Constants.CM3);
        backLeft = new Motor(hardwareMap, Constants.CM1);
        backRight = new Motor(hardwareMap, Constants.CM2);

        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        backLeft.setInverted(false);
        backRight.setInverted(false);

        //Create motor group with Front as the leader
        leftMotorGroup = new MotorGroup(this.frontLeft,this.backLeft);
        rightMotorGroup = new MotorGroup(this.frontRight,this.backRight);

//        rightMotorGroup.setInverted(true);

        // ----- Shooter Motors ----- //
        shooterLeft = new MotorEx(hardwareMap, Constants.EM3);
        shooterRight = new MotorEx(hardwareMap, Constants.EM2);
        shooterLeft.setInverted(true);

        shooterPair = new MotorExGroup(new ArrayList<>(Arrays.asList(this.shooterLeft, this.shooterRight)));
        shooterPair.setRunMode(Motor.RunMode.RawPower);
        shooterPair.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        // ----- Shooter Feed Motor ----- //
        tFeed = new Motor(hardwareMap, Constants.EM0);
        tFeed.setInverted(true);
        tFeed.setRunMode(Motor.RunMode.RawPower);

        // ----- Aim Servos ----- //
        turretAim = hardwareMap.get(Servo.class, Constants.CS2);
        hoodAim = hardwareMap.get(Servo.class, Constants.CS0);

        // ----- Intake ----- //
        intakeMotor = new Motor(hardwareMap, Constants.EM1);
        intakeMotor.setInverted(true);

        intakeBB = hardwareMap.get(DigitalChannel.class, Constants.CDI3);

        // ----- Kicker Servos ----- //
        kickerRotate = hardwareMap.get(Servo.class, Constants.CS1);
        kickerRotate.setPosition(0.0);
        sFeed = hardwareMap.get(CRServo.class, Constants.CS4);
        kickerSpin = hardwareMap.get(CRServo.class, Constants.CS3);


        // ----- Spindexer Servo ----- //
        spindexer = hardwareMap.get(Servo.class, Constants.CS5);

        // ----- Limelight ----- //
        limelight = hardwareMap.get(Limelight3A.class, Constants.LL);

        /* -------------- Gamepad -------------- */
        //Create the gamepad controller
        m_driveOp = new GamepadEx(gamepad1);

        /* -------------- IMU -------------- */
        turretIMU = hardwareMap.get(IMU.class, Constants.CBUS1);
        turretIMU.initialize(
                new IMU.Parameters(
                        new Rev9AxisImuOrientationOnRobot(
                                Rev9AxisImuOrientationOnRobot.LogoFacingDirection.UP,
                                Rev9AxisImuOrientationOnRobot.I2cPortFacingDirection.BACKWARD
                        )
                )
        );


        /* ---------- Pinpoint ----------*/
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.CBUS0);
        pinpoint.resetPosAndIMU();

        /* -------------- Sensors -------------- */
        topBreak = hardwareMap.get(DigitalChannel.class, Constants.CDI1);
        turretServoEncoder = new Motor(hardwareMap, Constants.EM0);


        /* -------------- Subsystems -------------- */
        /* 1. drive
         * 2. intake
         * 3. Kick
         * 4. Spindexer
         * 5. Limelight
         * 6. Shooter
         * 7. Aim*/

        //1. drive subsystem
        tankDriveSubsystem = new TankDriveSubsystem(leftMotorGroup,rightMotorGroup, pinpoint);
        //2. Intake subsystem
        intakeSubsystem = new IntakeSubsystem(intakeMotor, intakeBB);
        //3. Kick subsystem
        kickSubsystem = new KickSubsystem(kickerRotate, kickerSpin, sFeed);
        //4. Spindexer subsystem
        spindexerSubsystem = new SpindexerSubsystem(this, spindexer);
        //5. Limelight subsystem
        limelightSubsystem = new LimelightSubsystem(limelight, pipelineNum);
        //6. Shooter subsystem
        shooterSubsystem = new ShooterSubsystem(shooterPair, tFeed, topBreak);
        //7. Aim subsystem
        aimSubsystem = new AimSubsystem(hoodAim, turretAim, turretIMU, pinpoint, turretServoEncoder);



        /* -------------- COMMANDS -------------- */

            //Create a new drive command and pass in the drive subsystem and the gamepad control values
        //driveCmdTank = new DriveCmdTank(tankDriveSubsystem, m_driveOp::getLeftY, m_driveOp::getRightY);
            //Arcade Drive controls
        driveCmdArcade = new DriveCmdArcade(tankDriveSubsystem, m_driveOp::getLeftY, m_driveOp::getRightX);
        runIntakeCmd = new IntakeCmd(intakeSubsystem, spindexerSubsystem, kickSubsystem);
        stopIntakeCmd = new StopIntakeCmd(intakeSubsystem);
        uptakeCmd = new UptakeCmd(kickSubsystem);
        aimCmd = new FullAimToLLCmd(aimSubsystem, limelightSubsystem);






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
                .whenPressed(new SlotCmd(spindexerSubsystem, kickSubsystem, 1));

        m_driveOp.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new SlotCmd(spindexerSubsystem, kickSubsystem, 2));

        m_driveOp.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new SlotCmd(spindexerSubsystem, kickSubsystem, 3));


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

        aimSubsystem.aimTurret(Constants.TURRET_OFFSET);
        //aimSubsystem.aimTurret(0.0);
        sleep(250);
        while(aimSubsystem.turretBusy()){
            sleep(100);
        }
        aimSubsystem.resetIMU();
        sleep(250);
        turretServoEncoder.resetEncoder();


        aimSubsystem.aimHood(0.0);
        spindexerSubsystem.runToSlotOne();

        while (opModeIsActive()){
            //Scheduler must be loop called for everything else to run
            CommandScheduler.getInstance().run();
            pinpoint.update();


            //Right trigger press checking, if true, runs intake, else stops intake (may cause issues later if constantly scheduling stop...)
            if ( rightTriggerReader.isDown() ) {
                runIntakeCmd.schedule();
            } else {
                stopIntakeCmd.schedule();
            }

            if(!aimCmd.isScheduled()) {
                aimCmd.schedule();
            }

            //turretAim.setPosition((0.2*40.0)/(48.0));

            telemetry.addData("Limelight Tx: ", limelightSubsystem.getTx());
            telemetry.addData("Turret Degrees: ", aimSubsystem.getTurretServoDegrees());
//            telemetry.addData("Turret IMU Degrees: ", aimSubsystem.getHeadingAngles());
//            telemetry.addData("Turret IMU Without Offset: ", aimSubsystem.getHeadingAnglesWithoutOffset());
//            telemetry.addData("Robot IMU: ", aimSubsystem.getRobotHeading());
            telemetry.addData("encoder degrees: ", aimSubsystem.getEncoderDegrees());
            telemetry.addData("Slot: ", spindexerSubsystem.getSlot());
            telemetry.addData("Spindexer Pos: ", spindexer.getPosition());

            telemetry.addData("encoder ticks: ", turretServoEncoder.getCurrentPosition());
            telemetry.addData("servo command: ", turretAim.getPosition());

            //telemetry();
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

        telemetry.addData("Bay Number", spindexerSubsystem.getSlot());

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
