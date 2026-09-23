package org.firstinspires.ftc.team12841.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team12841.Drivers.RobotHardware;
import org.firstinspires.ftc.team12841.commands.FieldCentricCommand;
import org.firstinspires.ftc.team12841.commands.commandGroups.HangStepOne;
import org.firstinspires.ftc.team12841.commands.commandGroups.HangStepTwo;
import org.firstinspires.ftc.team12841.commands.liftcommands.LiftHighChamber;
import org.firstinspires.ftc.team12841.commands.liftcommands.LiftLowChamber;
import org.firstinspires.ftc.team12841.commands.liftcommands.LiftReset;
import org.firstinspires.ftc.team12841.commands.liftcommands.LiftScore;
import org.firstinspires.ftc.team12841.commands.liftcommands.LiftToHumanPlayer;
import org.firstinspires.ftc.team12841.commands.liftcommands.LiftTotalReset;
import org.firstinspires.ftc.team12841.commands.subextendcommands.SubExtendExtake;
import org.firstinspires.ftc.team12841.commands.subextendcommands.SubExtendFreeManipIn;
import org.firstinspires.ftc.team12841.commands.subextendcommands.SubExtendFreeManipOut;
import org.firstinspires.ftc.team12841.commands.subextendcommands.SubExtendHumanPlayer;
import org.firstinspires.ftc.team12841.commands.subextendcommands.SubExtendIn;
import org.firstinspires.ftc.team12841.commands.subextendcommands.SubExtendIntake;
import org.firstinspires.ftc.team12841.commands.subextendcommands.SubExtendOut;
import org.firstinspires.ftc.team12841.commands.subextendcommands.SubIntakeDown;
import org.firstinspires.ftc.team12841.commands.subextendcommands.SubIntakeUp;
import org.firstinspires.ftc.team12841.subsystems.DriveSubsystem;
import org.firstinspires.ftc.team12841.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.team12841.subsystems.LiftSubsystem;

import java.util.function.BooleanSupplier;

@TeleOp(name = "Teleop")
public class TeleopMain extends CommandOpMode {
    RobotHardware robot_;
    //declare subsystems
    DriveSubsystem m_drive;
    LiftSubsystem m_lift;
    IntakeSubsystem m_intake;

    //declare commands and command groups
    FieldCentricCommand driveCommand;
    HangStepOne hangStep1;
    HangStepTwo hangStep2;
    LiftReset liftReset;
    LiftHighChamber highChamber;
    LiftLowChamber lowChamber;
    LiftToHumanPlayer humanPlayer;
    LiftScore score;
    SubExtendOut subExtendOut;
    SubExtendIn subExtendIn;
    SubExtendIntake intake;
    SubExtendExtake extake;
    SubExtendFreeManipIn subExtendFreeManipIn;
    SubExtendFreeManipOut subExtendFreeManipOut;
    SubExtendHumanPlayer subExtendHumanPlayer;//aka spit
    LiftTotalReset liftTotalReset;
    SubIntakeDown subIntakeDown; //jae add move intake down
    SubIntakeUp subIntakeUp; //ae add move intake up

    ElapsedTime timer;


    //buttons - Named based on the which controller and button is used
    //TODO; determine what button does what
    Button manipLeftBumper;
    Button manipRightBumper;//score
    Button manipA;//hp
    Button manipB;//low chamber
    Button manipy;//high chamber
    Button manipX;//reset
    Button manipDpadUp;//hang s1
    Button manipDpadDown;//hang s2
    Button manipDpadRight;
    Button manipDpadLeft;
    Button manipGuide;

    Button driverLeftBumper;//intake
    Button driverRightBumper;//extake
    Button driverRightStick;//spit
    Button driverB;
    Button drivery;
    Button driverX;
    Button driverDpadUp;//subextend out
    Button driverDpadDown;//subextend in
    Button driverDpadRight;
    Button driverDpadLeft;


    BooleanSupplier rt;
    BooleanSupplier lt;

    Trigger driverRightTrigger;
    Trigger driverLeftTrigger;


    //declare controllers
   GamepadEx driverPad;//the driver's controller
   GamepadEx manipPad;
   boolean bucket = false;// the manipulator's controller

    @Override
    public void initialize() /*throws InterruptedException*/ {
        //Initialize robot - Motor, servos, IMU, Sensors
        robot_ = new RobotHardware(this);

        //Initialize subsystems
        m_drive = new DriveSubsystem(robot_.motorFrontLeft, robot_.motorFrontRight, robot_.motorBackLeft, robot_.motorBackRight, robot_.imu);
        m_lift = new LiftSubsystem(robot_.liftMotor, robot_.hangMotor, robot_.clawServo,robot_.hangServo,robot_.liftTouch, robot_.rightBase, robot_.leftBase);

         m_intake = new IntakeSubsystem(robot_.subExtendMotor, robot_.sampleServo, robot_.intakeServo/*, robot_.bucketColorSensor, robot_.belowColorSensor*/);


        //initialize commands and command groups
        driveCommand = new FieldCentricCommand(m_drive, robot_.OpMode_);
        hangStep1 = new HangStepOne(m_lift, robot_.blinkinLedDriver, robot_.OpMode_, m_intake);
        hangStep2 = new HangStepTwo(m_lift, robot_.blinkinLedDriver, robot_.OpMode_);
        liftReset = new LiftReset(m_lift, robot_.OpMode_);
        highChamber = new LiftHighChamber(m_lift, robot_.OpMode_);
        lowChamber = new LiftLowChamber(m_lift, robot_.OpMode_);
        humanPlayer = new LiftToHumanPlayer(m_lift, robot_.OpMode_);
        score = new LiftScore(m_lift, m_drive, robot_.OpMode_);
        liftTotalReset = new LiftTotalReset(m_lift, robot_.OpMode_);






        subExtendOut = new SubExtendOut(m_intake, robot_.OpMode_);
        subExtendIn = new SubExtendIn(m_intake, robot_.OpMode_);
        intake = new SubExtendIntake(m_intake, robot_.OpMode_);
        extake = new SubExtendExtake(m_intake, robot_.OpMode_);
        subExtendHumanPlayer = new SubExtendHumanPlayer(m_intake, robot_.OpMode_);
        subExtendFreeManipIn = new SubExtendFreeManipIn(m_intake, robot_.OpMode_, robot_);
        subExtendFreeManipOut = new SubExtendFreeManipOut(m_intake, robot_.OpMode_);


        //****************JAE ADD*****************
        subIntakeDown = new SubIntakeDown(m_intake, robot_.OpMode_);
        subIntakeUp = new SubIntakeUp(m_intake, robot_.OpMode_);
        //*****************JAE END ADD***************/

        //initialize controllers
        driverPad = new GamepadEx(gamepad1);
        manipPad = new GamepadEx(gamepad2);

        //initialize buttons

        manipRightBumper = (new GamepadButton(manipPad, GamepadKeys.Button.RIGHT_BUMPER));
        manipA = (new GamepadButton(manipPad, GamepadKeys.Button.A));
        manipB = (new GamepadButton(manipPad, GamepadKeys.Button.B));
        manipX = (new GamepadButton(manipPad, GamepadKeys.Button.X));
        manipy = (new GamepadButton(manipPad, GamepadKeys.Button.Y));
        manipDpadUp = (new GamepadButton(manipPad, GamepadKeys.Button.DPAD_UP));
        manipDpadDown = (new GamepadButton(manipPad, GamepadKeys.Button.DPAD_DOWN));


        //temp
        manipDpadRight = (new GamepadButton(manipPad, GamepadKeys.Button.DPAD_RIGHT));
        manipDpadLeft = (new GamepadButton(manipPad, GamepadKeys.Button.DPAD_LEFT));


        driverLeftBumper = (new GamepadButton(driverPad, GamepadKeys.Button.LEFT_BUMPER));
        driverRightBumper = (new GamepadButton(driverPad, GamepadKeys.Button.RIGHT_BUMPER));
        driverDpadUp = (new GamepadButton(driverPad, GamepadKeys.Button.DPAD_UP));
        driverDpadDown = (new GamepadButton(driverPad, GamepadKeys.Button.DPAD_DOWN));
        driverRightStick = (new GamepadButton(driverPad, GamepadKeys.Button.RIGHT_STICK_BUTTON));
        driverDpadLeft = (new GamepadButton(driverPad, GamepadKeys.Button.DPAD_LEFT)); //retract intake
        driverDpadRight = (new GamepadButton(driverPad, GamepadKeys.Button.DPAD_RIGHT)); //extend

        lt = () -> gamepad1.left_trigger > .75;
        rt = () -> gamepad1.right_trigger > .75;

        driverLeftTrigger = new Trigger(lt);
        driverRightTrigger = new Trigger(rt);

        //set commands to buttons

        manipRightBumper.whenPressed(score,true);// starts command when the button is pressed

        manipA.whenPressed(humanPlayer,true);
        manipB.whenPressed(lowChamber,true);
        manipX.whenPressed(liftReset,true);
        manipy.whenPressed(highChamber,true);
        manipDpadUp.whenPressed(hangStep1,true);
        manipDpadDown.whenPressed(hangStep2,true);

        timer = new ElapsedTime();




        driverLeftBumper.whileHeld(extake,true); //intake
        driverRightBumper.whileHeld(intake,true);//extake
        //driverDpadUp.whenPressed(subExtendOut,true);
        //driverDpadDown.whenPressed(subExtendIn,true);
        driverRightStick.whileHeld(subExtendHumanPlayer,true);

        driverRightTrigger.whileActiveContinuous(subExtendFreeManipOut,true);
        driverLeftTrigger.whileActiveContinuous(subExtendFreeManipIn,true);


        //------------------------JAE ADD---------------------*
        //driverDpadLeft.whileHeld(subExtendFreeManipIn,true);
        //driverDpadRight.whileHeld(subExtendFreeManipOut,true);
        driverDpadUp.whenPressed(subIntakeUp,true);
        driverDpadDown.whenPressed(subIntakeDown,true);
        //*------------------------JAE ADD---------------------

        //register subsystems
        register(m_drive);

        //set default command to the drive subsystem
        m_drive.setDefaultCommand(driveCommand);

        //m_intake.setDefaultCommand(subExtendFreeManip);

    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();
        timer.reset();
        m_intake.setIntakeDropServo(RobotHardware.INTAKE_SERVO_UP);


        // run the scheduler
        //TODO; add telemetry into here if not already in commands or subsystems
        while (!isStopRequested() && opModeIsActive()) {
            run();

            //telemetry.addData("subextend postion = ",robot_.subExtendMotor.getCurrentPosition()* RobotHardware.SUBEXTEND_COUNTS_PER_INCH);
            telemetry.update();

            if(gamepad1.guide){
                robot_.imu.resetYaw();
            }

            if(gamepad2.guide){
                liftTotalReset.initialize();
            }

            if(hangStep1.hangDone){
                robot_.blinkinLedDriver.setPattern(BlinkinPattern.RAINBOW_RAINBOW_PALETTE);

            }else if(driveCommand.babyMode){
                robot_.blinkinLedDriver.setPattern(BlinkinPattern.HOT_PINK);
            }else if(timer.seconds() > 90 && timer.seconds() < 100){
                robot_.blinkinLedDriver.setPattern(BlinkinPattern.BLUE_VIOLET);
            }else if(timer.seconds() > 100 && timer.seconds() < 110){
                robot_.blinkinLedDriver.setPattern(BlinkinPattern.STROBE_GOLD);
            }else if(timer.seconds() >= 110 && timer.seconds() < 120){
                robot_.blinkinLedDriver.setPattern(BlinkinPattern.STROBE_RED);
            }else if(timer.seconds() >= 120){
                robot_.blinkinLedDriver.setPattern(BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
            }else {
                robot_.blinkinLedDriver.setPattern(BlinkinPattern.CP1_2_BEATS_PER_MINUTE);
            }


            //remove later once we figure out implementation
//            if(-manipPad.getRightY() < 0 && robot_.subTouch.isPressed()){
//
//                robot_.subExtendMotor.setPower(0);
//            }else {
//                robot_.subExtendMotor.setPower(-manipPad.getRightY());
//            }

            if(!gamepad1.left_bumper && !gamepad1.right_bumper){
                m_intake.intakeOff();

            }











        }
        reset();
    }

}
