package org.firstinspires.ftc.team12841.drive.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.drive.SampleMecanumDrive;

@Disabled
@Autonomous(group = "drive")
public class LocationTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap); //pull in drive to use


        //stuff so it doesn't get funky (I think this tells the brick to stop when it says stop and go when it says go)
        waitForStart();
        if (isStopRequested()) return;

        Trajectory forward = drive.trajectoryBuilder(new Pose2d(48, 48, Math.toRadians(90)))
                .forward(1)
                .build();

        //set start pos
        drive.setPoseEstimate(new Pose2d(48, 48, Math.toRadians(90)));

        drive.followTrajectory(forward);

        sleep(10000);

    }
}
