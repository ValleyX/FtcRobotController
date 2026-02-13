package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Helper.Subsystems;
import org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.Roadrunner.MecanumDrive;

public class BlueClose extends CommandOpMode {
    Subsystems subsystems;

    @Override
    public void initialize() {
        subsystems = new Subsystems(hardwareMap, 0, true);
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(0.0, 0.0, Math.toRadians(45));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

    }


    @Override
    public void runOpMode(){

    }
}
