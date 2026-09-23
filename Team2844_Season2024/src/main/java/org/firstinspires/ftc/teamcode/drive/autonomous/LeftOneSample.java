package org.firstinspires.ftc.team12841.drive.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.Drivers.RobotHardware;
import org.firstinspires.ftc.team12841.commands.autocommands.AutoScoreHigh;
import org.firstinspires.ftc.team12841.commands.liftcommands.LiftHighChamber;
import org.firstinspires.ftc.team12841.commands.liftcommands.LiftTotalReset;
import org.firstinspires.ftc.team12841.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team12841.subsystems.LiftSubsystem;

@Autonomous(group = "drive")
public class LeftOneSample extends LinearOpMode {
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


        //CREATE TRAJECTORIES
        Trajectory forward = drive.trajectoryBuilder(new Pose2d(-8, -62, Math.toRadians(90)))
                .forward(20)
                .build();

        Trajectory forward2 = drive.trajectoryBuilder(forward.end())
                .forward(11)
                .build();

        Trajectory back = drive.trajectoryBuilder(forward2.end())
                .forward(-24)
                .build();

        Trajectory strafeOver = drive.trajectoryBuilder(back.end())
                .strafeLeft(24)
                .build();

        Trajectory splineToSamples = drive.trajectoryBuilder(strafeOver.end())
                .splineTo(new Vector2d(-36, -16), Math.toRadians(90))
                .build();

        Trajectory orient1 = drive.trajectoryBuilder(splineToSamples.end())
                .strafeLeft(6)
                .build();

        Trajectory sample1A = drive.trajectoryBuilder(orient1.end())
                .splineToConstantHeading(new Vector2d(-60, -54), Math.toRadians(90))
                .build();

        Trajectory sample1B = drive.trajectoryBuilder(sample1A.end())
                .splineToConstantHeading(new Vector2d(-42, -12), Math.toRadians(90))
                .build();

        Trajectory orient2 = drive.trajectoryBuilder(sample1B.end())
                .strafeLeft(12)
                .build();

        Trajectory sample2A = drive.trajectoryBuilder(orient2.end())
                .splineToConstantHeading(new Vector2d(-60, -54), Math.toRadians(90))
                .build();

        Trajectory sample2B = drive.trajectoryBuilder(sample2A.end())
                .splineToConstantHeading(new Vector2d(-58, -12), Math.toRadians(90))
                .build();

        Trajectory orient3 = drive.trajectoryBuilder(sample2B.end())
                .strafeLeft(6)
                .build();

        Trajectory sample3A = drive.trajectoryBuilder(orient3.end())
                .splineToConstantHeading(new Vector2d(-60, -54), Math.toRadians(90))
                .build();



        //SET START POS
        drive.setPoseEstimate(new Pose2d(-8, -62, Math.toRadians(90)));



        //EXECUTE TRAJECTORIES
        sleep(250);

        //engage servo at beginning
        m_liftSub.bucketToPosition(RobotHardware.CLAW_SERVO_ENGAGED);

        //move lift up
        highChamber.initialize();
        sleep(250);

        //follow trajectories
        drive.followTrajectory(forward);
        sleep(50);

        //move into scoring position
        drive.followTrajectory(forward2);
        sleep(250);

        //score high chamber
        autoScoreHigh.initialize();
        sleep(200);

        drive.followTrajectory(back);
        sleep(50);

        //resets lift
        liftReset.initialize();
        sleep(100);

        //moves to the side a bit
        drive.followTrajectory(strafeOver);
        sleep(50);

        //drives to samples using a spline
        drive.followTrajectory(splineToSamples);
        sleep(50);

        //orients to first sample
        drive.followTrajectory(orient1);
        sleep(50);

        //scores first sample
        drive.followTrajectory(sample1A);
        sleep(50);

        //goes back for 2nd sample
        drive.followTrajectory(sample1B);
        sleep(50);

        //orients for 2nd sample score
        drive.followTrajectory(orient2);
        sleep(50);

        //score 2nd sample
        drive.followTrajectory(sample2A);
        sleep(50);

        //goes back for 3rd sample
        drive.followTrajectory(sample2B);
        sleep(50);

        //orients for 3rd sample score
        drive.followTrajectory(orient3);
        sleep(50);

        //score 3rd sample
        drive.followTrajectory(sample3A);
        sleep(50);

    }
}
