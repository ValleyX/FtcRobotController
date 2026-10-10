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

    public final DriveSubsystem drive;
    public final IntakeSubsystem intake;
    public final LightSubsystem light;
    public final ShooterSubsystem nectar;
    public final ShooterSubsystem pollen;
    public final TurretSubsystem turret;
    public final VisionSubsystem vision;

    Constants.ShooterConfig nectarConfig, pollenConfig;

    public Subsystems(HardwareMap hardwareMap, Pose startingPose, int pipeline){

        nectarConfig = new Constants.ShooterConfig(
                Constants.EHM2,
                Constants.NP,
                Constants.NI,
                Constants.ND,
                Constants.NS,
                Constants.NV
        );
        pollenConfig = new Constants.ShooterConfig(
                Constants.EHM3,
                Constants.PP,
                Constants.PI,
                Constants.PD,
                Constants.PS,
                Constants.PV
        );

        drive = new DriveSubsystem(hardwareMap, startingPose);
        intake = new IntakeSubsystem(hardwareMap);
        light = new LightSubsystem(hardwareMap);
        nectar = new ShooterSubsystem(hardwareMap, nectarConfig);
        pollen = new ShooterSubsystem(hardwareMap, pollenConfig);
        turret = new TurretSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);

        vision.setPipeline(pipeline);
    }

}
