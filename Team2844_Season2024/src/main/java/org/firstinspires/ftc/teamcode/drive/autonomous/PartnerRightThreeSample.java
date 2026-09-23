package org.firstinspires.ftc.team12841.drive.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.Drivers.RobotHardware;
import org.firstinspires.ftc.team12841.commands.autocommands.AutoScoreHigh;
import org.firstinspires.ftc.team12841.commands.liftcommands.LiftHighChamber;
import org.firstinspires.ftc.team12841.commands.liftcommands.LiftTotalReset;
import org.firstinspires.ftc.team12841.commands.liftcommands.LiftToHumanPlayer;
import org.firstinspires.ftc.team12841.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team12841.subsystems.LiftSubsystem;

@Disabled
@Autonomous(group = "drive") //makes it an autonomous program
public class PartnerRightThreeSample extends LinearOpMode {
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap); //pull in drive to use
        RobotHardware robot = new RobotHardware(this); //make an instance of robot hardware
        LiftSubsystem m_liftSub = new LiftSubsystem(robot.liftMotor, robot.hangMotor, robot.clawServo, robot.hangServo, robot.liftTouch, robot.rightBase, robot.leftBase); //making an instance of liftSub

        AutoScoreHigh autoScoreHigh = new AutoScoreHigh(m_liftSub, this);
        LiftTotalReset liftReset = new LiftTotalReset(m_liftSub,this);
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
                .forward(10.5)
                .build();

        //EXPERIMENTAL
        Trajectory splineToSamplesEx = drive.trajectoryBuilder(forward2.end())
                //.splineToConstantHeading(new Vector2d(14, -18), Math.toRadians(90))
                //.splineToConstantHeading(new Vector2d(24, -24), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(30, -24), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(38, -12), Math.toRadians(90))
                .build();

//        Trajectory back1 = drive.trajectoryBuilder(new Pose2d(12, -29, Math.toRadians(90)))
//                .forward(-6)
//                .build();
//
//        Trajectory sideways = drive.trajectoryBuilder(back1.end())
//                .strafeRight(18)
//                .build();
//
//        Trajectory splineToSamples = drive.trajectoryBuilder(sideways.end())
//                .splineTo(new Vector2d(38, -12), Math.toRadians(90))
//                .forward(2)
//                .build();

        Trajectory orient1 = drive.trajectoryBuilder(splineToSamplesEx.end())
                .strafeRight(6)
                .build();

        Trajectory score1A = drive.trajectoryBuilder(orient1.end())
                .back(42)
                .build();

        //strafe over to a pickup zone
        Trajectory pickup1 = drive.trajectoryBuilder(score1A.end())
                .splineTo(new Vector2d(36, -61.5), Math.toRadians(270))
                .build();
        //pickup the sample
            //Call command for this crap
        //back up to avoid wall
        Trajectory noScrape2 = drive.trajectoryBuilder(pickup1.end())
                .back(10)
                .build();
        //spline to score sample
        Trajectory splineToScore = drive.trajectoryBuilder(noScrape2.end())
                .splineTo(new Vector2d(4, -27.5), Math.toRadians(90))
                .build();
        //score sample
            //call command for this crap

        Trajectory backAgain = drive.trajectoryBuilder((splineToScore.end()))
                .back(10)
                .build();

        Trajectory pickup2 = drive.trajectoryBuilder(backAgain.end())
                .splineTo(new Vector2d(36, -61.5), Math.toRadians(270))
                .build();

        Trajectory noScrape3 = drive.trajectoryBuilder(pickup2.end())
                .back(10)
                .build();

        //spline to score sample
        Trajectory splineToScore2 = drive.trajectoryBuilder(noScrape3.end())
                .splineTo(new Vector2d(0, -27.5), Math.toRadians(90))
                .build();

        //park
        Trajectory splineToPark = drive.trajectoryBuilder(splineToScore2.end())
                .splineToConstantHeading(new Vector2d(36, -56), Math.toRadians(90))
                .build();


        //set start pos
        drive.setPoseEstimate(new Pose2d(7, -60, Math.toRadians(90)));

        //sleep(250);

        m_liftSub.bucketToPosition(RobotHardware.CLAW_SERVO_ENGAGED);

        //move lift up
        highChamber.initialize();
        sleep(350);

        //follow trajectories
        drive.followTrajectory(forward);
        sleep(10);

        //move into scoring position
        drive.followTrajectory(forward2);
        sleep(10);

        //score high chamber
        autoScoreHigh.initialize();
        sleep(100);

        //resets lift
        liftReset.initialize();

//        drive.followTrajectory(back1);
//        drive.followTrajectory(sideways);

        drive.followTrajectory(splineToSamplesEx);

        drive.followTrajectory(orient1);

        drive.followTrajectory(score1A);

        drive.followTrajectory(pickup1);

        //pickup the sample
        sleep(750);
        humanPlayer.initialize();
        sleep(100);
        highChamber.initialize();

        //spline to score sample
        drive.followTrajectory(noScrape2);
        drive.followTrajectory(splineToScore);

        //score sample
        sleep(500);
        autoScoreHigh.initialize();
        sleep(50);
        liftReset.initialize();

        //drive to pickup zone
        drive.followTrajectory(backAgain);
        drive.followTrajectory(pickup2);

        //pickup the sample
        sleep(750);
        humanPlayer.initialize();
        sleep(100);
        highChamber.initialize();

        //spline to score sample
        drive.followTrajectory(noScrape3);
        drive.followTrajectory(splineToScore2);

        //score sample
        sleep(500);
        autoScoreHigh.initialize();
        sleep(50);
        liftReset.initialize();

        //park
        drive.followTrajectory(splineToPark);


    }
}
