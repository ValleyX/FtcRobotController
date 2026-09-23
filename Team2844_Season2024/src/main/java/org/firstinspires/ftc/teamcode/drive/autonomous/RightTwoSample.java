package org.firstinspires.ftc.team12841.drive.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.Drivers.RobotHardware;
import org.firstinspires.ftc.team12841.commands.autocommands.AutoScoreHigh;
import org.firstinspires.ftc.team12841.commands.liftcommands.LiftHighChamber;
import org.firstinspires.ftc.team12841.commands.liftcommands.LiftReset;
import org.firstinspires.ftc.team12841.commands.liftcommands.LiftToHumanPlayer;
import org.firstinspires.ftc.team12841.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team12841.subsystems.LiftSubsystem;


@Autonomous(group = "drive") //makes it an autonomous program
public class RightTwoSample extends LinearOpMode {
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap); //pull in drive to use
        RobotHardware robot = new RobotHardware(this); //make an instance of robot hardware
        LiftSubsystem m_liftSub = new LiftSubsystem(robot.liftMotor, robot.hangMotor, robot.clawServo, robot.hangServo, robot.liftTouch, robot.rightBase, robot.leftBase); //making an instance of liftSub

        AutoScoreHigh autoScoreHigh = new AutoScoreHigh(m_liftSub, this);
        LiftReset liftReset = new LiftReset(m_liftSub,this);
        LiftHighChamber highChamber = new LiftHighChamber(m_liftSub,this);
        LiftToHumanPlayer humanPlayer = new LiftToHumanPlayer(m_liftSub, this);

        //stuff so it doesn't get funky (I think this tells the brick to stop when it says stop and go when it says go)
        waitForStart();
        if (isStopRequested()) return;

        //LED stuff
        //robot.blinkinLedDriver.setPattern(robot.bluePattern);


        //make trajectories
//        Trajectory traj = drive.trajectoryBuilder(new Pose2d(0, 0, 0)) //start position
//                .splineTo(new Vector2d(0, 0), 0) //finishing position
//                .build(); //builds command

        Trajectory forward = drive.trajectoryBuilder(new Pose2d(7, -60, Math.toRadians(90)))
                .forward(20)
                .build();

        Trajectory forward2 = drive.trajectoryBuilder(new Pose2d(7, -40, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(3, -27), Math.toRadians(90))
                .build();

        Trajectory back1 = drive.trajectoryBuilder(forward2.end())
                .forward(-6)
                .build();

        Trajectory sideways = drive.trajectoryBuilder(back1.end())
                .strafeRight(32)
                .build();

        Trajectory splineToSamples = drive.trajectoryBuilder(sideways.end())
                .splineTo(new Vector2d(38, -12), Math.toRadians(90))
                .forward(2)
                .build();

        Trajectory orient1 = drive.trajectoryBuilder(splineToSamples.end())
                .strafeRight(6)
                .build();

        Trajectory score1A = drive.trajectoryBuilder(orient1.end())
                .back(42)
                .build();

        Trajectory score1B = drive.trajectoryBuilder(score1A.end())
                .forward(42)
                .build();

        Trajectory orient2 = drive.trajectoryBuilder(score1B.end())
                .strafeRight(9)
                .build();

        Trajectory score2A = drive.trajectoryBuilder(orient2.end())
                .back(40)
                .build();

        //back up from wall to prevent scrape
//        Trajectory noScrape = drive.trajectoryBuilder(score2A.end())
//                .forward(0)
//                .build();
        //strafe over to a pickup zone
        Trajectory pickup1 = drive.trajectoryBuilder(score2A.end())
                .splineTo(new Vector2d(36, -62), Math.toRadians(270))
                .build();
        //pickup the sample
            //Call command for this crap
        //back up to avoid wall
        Trajectory noScrape2 = drive.trajectoryBuilder(pickup1.end())
                .back(10)
                .build();
        //spline to score sample
        Trajectory splineToScore = drive.trajectoryBuilder(noScrape2.end())
                .splineTo(new Vector2d(5, -28), Math.toRadians(90))
                .build();
        //score sample
            //call command for this crap

        //park
        Trajectory splineToPark = drive.trajectoryBuilder(splineToScore.end())
                .splineToConstantHeading(new Vector2d(36, -56), Math.toRadians(90))
                .build();


        //set start pos
        drive.setPoseEstimate(new Pose2d(7, -60, Math.toRadians(90)));

        //sleep(250);

        m_liftSub.bucketToPosition(RobotHardware.CLAW_SERVO_ENGAGED);

        //move lift up
        highChamber.initialize();
        sleep(200);

        //follow trajectories
        drive.followTrajectory(forward);
        sleep(10);

        //move into scoring position
        drive.followTrajectory(forward2);
        sleep(10);

        //score high chamber
        autoScoreHigh.initialize();
        sleep(100);

        liftReset.initialize();

        drive.followTrajectory(back1);
        //sleep(10);
        drive.followTrajectory(sideways);
        //sleep(10);

        //resets lift
        //liftReset.initialize();
        //sleep(100);

        drive.followTrajectory(splineToSamples);

        drive.followTrajectory(orient1);

        drive.followTrajectory(score1A);

        drive.followTrajectory(score1B);

        drive.followTrajectory(orient2);

        drive.followTrajectory(score2A);

        drive.followTrajectory(pickup1);

        //pickup the sample
        sleep(250);
        humanPlayer.initialize();
        sleep(100);
        highChamber.initialize();


        //spline to score sample
        drive.followTrajectory(noScrape2);
        drive.followTrajectory(splineToScore);

        //score sample
        sleep(150);
        autoScoreHigh.initialize();
        sleep(50);
        liftReset.initialize();
        m_liftSub.m_bucket.setPosition(RobotHardware.CLAW_SERVO_ENGAGED);

        //park
        drive.followTrajectory(splineToPark);


    }
}
