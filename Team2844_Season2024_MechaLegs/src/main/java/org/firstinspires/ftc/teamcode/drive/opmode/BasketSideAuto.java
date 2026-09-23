package org.firstinspires.ftc.team12841.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.drive.SampleMecanumDrive;

@Autonomous(group = "drive")
public class BasketSideAuto extends LinearOpMode {
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap); //pull in drive to use

        //stuff so it doesn't get funky (I think this tells the brick to stop when it says stop and go when it says go)
        waitForStart();
        if (isStopRequested()) return;

        //CREATE TRAJECTORIES
        Trajectory forward = drive.trajectoryBuilder(new Pose2d(7, 60, Math.toRadians(270)))
                .forward(24)
                .build();

        Trajectory back = drive.trajectoryBuilder(forward.end())
                .forward(-24)
                .build();

//        Trajectory park = drive.trajectoryBuilder(back.end()) //NO POINTS FOR PARKING IN NET ZONE, DO LVL1 "HANG" 4 POINTS
//                .strafeTo(new Vector2d(60, 60))
//                .build();

        //SET START POS
        drive.setPoseEstimate(new Pose2d(7, 60, Math.toRadians(270)));

        //EXECUTE TRAJECTORIES
        drive.followTrajectory(forward);
        sleep(50);
        drive.followTrajectory(back);
//        sleep(50);                        //NO POINTS FOR PARKING IN NET ZONE, DO LVL1 "HANG" 4 POINTS
//        drive.followTrajectory(park);

    }
}
