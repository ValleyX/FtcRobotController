package org.firstinspires.ftc.team12841.drive.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.Drivers.RobotHardware;
import org.firstinspires.ftc.team12841.commands.autocommands.AutoScoreHigh;
import org.firstinspires.ftc.team12841.commands.autocommands.DropIntake;
import org.firstinspires.ftc.team12841.commands.autocommands.IntakeSpit;
import org.firstinspires.ftc.team12841.commands.autocommands.RaiseIntake;
import org.firstinspires.ftc.team12841.commands.autocommands.SubToLength;
import org.firstinspires.ftc.team12841.commands.liftcommands.LiftHighChamber;
import org.firstinspires.ftc.team12841.commands.liftcommands.LiftReset;
import org.firstinspires.ftc.team12841.commands.liftcommands.LiftToHumanPlayer;
import org.firstinspires.ftc.team12841.commands.liftcommands.LiftTotalReset;
import org.firstinspires.ftc.team12841.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team12841.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.team12841.subsystems.LiftSubsystem;
import org.opencv.core.Mat;

//@Disabled
@Autonomous(group = "drive") //makes it an autonomous program
public class RightThreeSample extends LinearOpMode {
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap); //pull in drive to use
        RobotHardware robot = new RobotHardware(this); //make an instance of robot hardware
        LiftSubsystem m_liftSub = new LiftSubsystem(robot.liftMotor, robot.hangMotor, robot.clawServo, robot.hangServo, robot.liftTouch, robot.rightBase, robot.leftBase); //making an instance of liftSub

        AutoScoreHigh autoScoreHigh = new AutoScoreHigh(m_liftSub, this);
        LiftReset liftReset = new LiftReset(m_liftSub,this);
        LiftHighChamber highChamber = new LiftHighChamber(m_liftSub,this);
        LiftToHumanPlayer humanPlayer = new LiftToHumanPlayer(m_liftSub, this);
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(robot.subExtendMotor, robot.sampleServo, robot.intakeServo);
        SubToLength subToLength = new SubToLength(intakeSubsystem,this);
        DropIntake dropIntake = new DropIntake(intakeSubsystem, this);
        RaiseIntake raiseIntake = new RaiseIntake(intakeSubsystem, this);
        IntakeSpit intakeSpit = new IntakeSpit(intakeSubsystem, this);

        //stuff so it doesn't get funky (I think this tells the brick to stop when it says stop and go when it says go)
        waitForStart();
        if (isStopRequested()) return;


        //create Trajectory paths
        Trajectory forward = drive.trajectoryBuilder(new Pose2d(7, -60, Math.toRadians(90)))
                .forward(20)
                .build();

        //go up to score on submersible
        Trajectory forward2 = drive.trajectoryBuilder(forward.end())
                .splineToConstantHeading(new Vector2d(3, -27), Math.toRadians(90))
                .build();

        //back up from submersible
        Trajectory back1 = drive.trajectoryBuilder(forward2.end())
                .forward(-10)
                .build();

        //strafes to clear zone next to submersible
        Trajectory orient1 = drive.trajectoryBuilder(back1.end())
                //.strafeRight(32)
                .splineTo(new Vector2d(36, -40), Math.toRadians(0))
                .build();

        //drives forward to line up for sample pickup
        Trajectory orient2 = drive.trajectoryBuilder(orient1.end())
                .strafeLeft(15)
                .build();

        //move to drop off brick
        Trajectory orient3 = drive.trajectoryBuilder(new Pose2d(51,-40), Math.toRadians(-65))
                .splineTo(new Vector2d(57,-55),Math.toRadians(-65))
                .build();

        Trajectory orient4 = drive.trajectoryBuilder(orient3.end())
                .splineToConstantHeading(new Vector2d(48, -28), Math.toRadians(0))
                .build();

//        Trajectory orient5 = drive.trajectoryBuilder(orient4.end())
//                .forward(7)
//                .build();



        //set start pos
        drive.setPoseEstimate(new Pose2d(7, -60, Math.toRadians(90)));

        //engage claw servo
        m_liftSub.bucketToPosition(RobotHardware.CLAW_SERVO_ENGAGED);

        //move lift up
        highChamber.initialize();
        sleep(600);

        //follow trajectories
        drive.followTrajectory(forward);
        sleep(10);

        //move into scoring position
        drive.followTrajectory(forward2);
        sleep(10);

        //score high chamber
        autoScoreHigh.initialize();
        sleep(100);

        drive.followTrajectory(back1);
        sleep(10);

        //resets lift
        liftReset.initialize();

        //gets in range of first sample
        drive.followTrajectory(orient1);
        drive.followTrajectory(orient2);
//        drive.followTrajectory(orient3);

        //go go gadget subextend out
        subToLength.initialize(9);

        //start intake
        dropIntake.initialize();

        //pause so that the sample has time to be intaked intook? whatever that word is
        sleep(800); //600

        //stop intake
        raiseIntake.initialize();

        //fully extend sub extend
        subToLength.initialize(20);

        //turn to hp
        drive.turn(Math.toRadians(-65));




        //strafe to get closer to the obs zone
        drive.followTrajectory(orient3);

        /*
        //go go gadget subextend out (but more to score)
        subToLength.initialize(20);
        sleep(200);

        //turn to score
        drive.turn(Math.toRadians(-65));
        sleep(400);

        //spit out sample
        intakeSpit.initialize();

        //retract subextend
        subToLength.initialize(0);

        //turn back to a straight heading
        drive.turn(Math.toRadians(65));

        //get over to next sample
        drive.followTrajectory(orient4);
        sleep(500);

        //pickup routine------------------
        subToLength.initialize(6);
        //start intake
        dropIntake.initialize();
        //pause so that the sample has time to be intaked intook? whatever that word is
        sleep(800); //600
        //stop intake
        raiseIntake.initialize();

        //turn to face obs zone
        drive.turn(Math.toRadians(-90));

        //extend out for drop off
        subToLength.initialize(20);

        //spit out sample
        intakeSpit.initialize();*/




    }
}
