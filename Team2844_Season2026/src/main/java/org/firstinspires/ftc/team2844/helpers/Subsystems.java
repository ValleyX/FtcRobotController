package org.firstinspires.ftc.team2844.helpers;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team2844.subsystems.DriveSubsystem;
import org.firstinspires.ftc.team2844.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.subsystems.LightSubsystem;
import org.firstinspires.ftc.team2844.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.team2844.subsystems.TransferSubsystem;
import org.firstinspires.ftc.team2844.subsystems.TurretSubsystem;
import org.firstinspires.ftc.team2844.subsystems.VisionSubsystem;

import java.util.List;

public class Subsystems {

    public final DriveSubsystem drive;
    public final IntakeSubsystem intake;
    public final LightSubsystem light;
    public final ShooterSubsystem nectar;
    public final ShooterSubsystem pollen;
    public final TransferSubsystem transfer;
    public final TurretSubsystem turret;
    public final VisionSubsystem vision;

    Constants.ShooterConfig nectarConfig, pollenConfig;

    public Subsystems(HardwareMap hardwareMap, Pose startingPose, int pipeline){

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hubs : allHubs){
            hubs.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        nectarConfig = new Constants.ShooterConfig(
                Constants.EHM2,
                Constants.CHS0,
                Constants.NP,
                Constants.NI,
                Constants.ND,
                Constants.NS,
                Constants.NV
        );
        pollenConfig = new Constants.ShooterConfig(
                Constants.EHM3,
                Constants.CHS1,
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
        transfer = new TransferSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);

        vision.setPipeline(pipeline);
    }

    public double distance(){
        double driveDis = drive.getDistance();
        double visDis = vision.getDistance();
        if(driveDis != Constants.NO_DIS && visDis != Constants.NO_DIS){
            return (driveDis + visDis)/2.0;
        } else if(driveDis != Constants.NO_DIS){
            return driveDis;
        } else if(visDis != Constants.NO_DIS){
            return visDis;
        } else {
            return -999.0;
        }
    }

    public void update(){
        nectar.setDistance(distance());
        pollen.setDistance(distance());
        turret.setPose(drive.getPose().x(), drive.getPose().y(), drive.getNormalHeadingDegrees());
        transfer.setBallCount(intake.ballCount());
    }

}
