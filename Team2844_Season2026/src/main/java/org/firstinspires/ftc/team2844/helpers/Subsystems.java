package org.firstinspires.ftc.team2844.helpers;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team2844.subsystems.DriveSubsystem;
import org.firstinspires.ftc.team2844.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.subsystems.LightSubsystem;
import org.firstinspires.ftc.team2844.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.team2844.subsystems.TurretSubsystem;
import org.firstinspires.ftc.team2844.subsystems.VisionSubsystem;

public class Subsystems {

    DriveSubsystem drive;
    IntakeSubsystem intake;
    LightSubsystem light;
    ShooterSubsystem shooter;
    TurretSubsystem turret;
    VisionSubsystem vision;

    public Subsystems(HardwareMap hardwareMap, Pose startingPose, int pipeline){
        drive = new DriveSubsystem(hardwareMap, startingPose);
        intake = new IntakeSubsystem(hardwareMap);
        light = new LightSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);

        vision.setPipeline(pipeline);
    }

}
