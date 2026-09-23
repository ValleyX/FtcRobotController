package org.firstinspires.ftc.team12841.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.drive.SampleMecanumDrive;

@Autonomous(group = "drive")
public class AutoTesting extends LinearOpMode {
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap); //pull in drive to use

        //stuff so it doesn't get funky (I think this tells the brick to stop when it says stop and go when it says go)
        waitForStart();
        if (isStopRequested()) return;


        //make trajectories
        Trajectory traj = drive.trajectoryBuilder(new Pose2d()) //start position
                .splineTo(new Vector2d(60, 10), 0) //finishing position
                .build(); //builds command

        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(60, 10, Math.toRadians(0)), Math.toRadians(90))
                .splineTo(new Vector2d(0, 0), Math.toRadians(90))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d(60, 10, Math.toRadians(90)))
                .strafeLeft(60)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end().plus(new Pose2d(0, 0, Math.toRadians(0))))
                .forward(-50)
                .build();


        //set start pos estimate
        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        //follow trajectories
        drive.followTrajectory(traj);
        drive.turn(Math.toRadians(90));
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);

    }
}
