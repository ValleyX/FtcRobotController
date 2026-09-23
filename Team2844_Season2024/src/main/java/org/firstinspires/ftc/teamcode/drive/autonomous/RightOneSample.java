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
import org.firstinspires.ftc.team12841.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team12841.subsystems.LiftSubsystem;

@Disabled
@Autonomous(group = "drive") //makes it an autonomous program
public class RightOneSample extends LinearOpMode {
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap); //pull in drive to use
        RobotHardware robot = new RobotHardware(this); //make an instance of robot hardware
        LiftSubsystem m_liftSub = new LiftSubsystem(robot.liftMotor, robot.hangMotor, robot.clawServo, robot.hangServo, robot.liftTouch, robot.rightBase, robot.leftBase); //making an instance of liftSub

        AutoScoreHigh autoScoreHigh = new AutoScoreHigh(m_liftSub, this);
        LiftTotalReset liftReset = new LiftTotalReset(m_liftSub,this);
        LiftHighChamber highChamber = new LiftHighChamber(m_liftSub,this);

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
                .forward(11)
                .build();

        Trajectory back1 = drive.trajectoryBuilder(new Pose2d(12, -29, Math.toRadians(90)))
                .forward(-12)
                .build();

        Trajectory sideways = drive.trajectoryBuilder(back1.end())
                .strafeRight(18)
                .build();

        Trajectory splineToSamples = drive.trajectoryBuilder(sideways.end())
                .splineTo(new Vector2d(36, -12), Math.toRadians(90))
                .forward(6)
                .build();

        Trajectory orient1 = drive.trajectoryBuilder(splineToSamples.end())
                .strafeRight(6)
                .build();

        Trajectory score1A = drive.trajectoryBuilder(orient1.end())
                .back(52)
                .build();

        Trajectory score1B = drive.trajectoryBuilder(score1A.end())
                .forward(52)
                .build();

        Trajectory orient2 = drive.trajectoryBuilder(score1B.end())
                .strafeRight(9)
                .build();

        Trajectory score2A = drive.trajectoryBuilder(orient2.end())
                .back(50)
                .build();

        Trajectory score2B = drive.trajectoryBuilder(score2A.end())
                .forward(50)
                .build();

        Trajectory orient3 = drive.trajectoryBuilder(score2B.end())
                .strafeRight(9)
                .build();

        Trajectory score3 = drive.trajectoryBuilder(orient3.end())
                .back(46)
                .build();


        //set start pos
        drive.setPoseEstimate(new Pose2d(7, -60, Math.toRadians(90)));

        sleep(250);

        m_liftSub.bucketToPosition(RobotHardware.CLAW_SERVO_ENGAGED);

        //move lift up
        highChamber.initialize();
        sleep(250);

        //follow trajectories
        drive.followTrajectory(forward);
        sleep(50);

        //move into scoring position
        drive.followTrajectory(forward2);
        sleep(100);

        //score high chamber
        autoScoreHigh.initialize();
        sleep(200);

        drive.followTrajectory(back1);
        sleep(50);
        drive.followTrajectory(sideways);
        sleep(25);

        //resets lift
        liftReset.initialize();
        sleep(100);

        drive.followTrajectory(splineToSamples);
        sleep(25);
        drive.followTrajectory(orient1);
        sleep(25);
        drive.followTrajectory(score1A);
        sleep(25);
        drive.followTrajectory(score1B);
        sleep(25);
        drive.followTrajectory(orient2);
        sleep(25);
        drive.followTrajectory(score2A);
        sleep(25);
        drive.followTrajectory(score2B);
        sleep(25);
        drive.followTrajectory(orient3);
        sleep(25);
        drive.followTrajectory(score3);


    }
}
