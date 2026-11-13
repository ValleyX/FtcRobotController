package org.firstinspires.ftc.team2844simple.helpers;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team2844simple.subsystems.DriveSubsystem;
import org.firstinspires.ftc.team2844simple.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844simple.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.team2844simple.subsystems.TransferSubsystem;
import org.firstinspires.ftc.team2844simple.subsystems.VisionSubsystem;

import java.util.List;

public class Subsystems {

    public final DriveSubsystem drive;
    public final IntakeSubsystem intake;
    public final ShooterSubsystem pollen;
    public final TransferSubsystem transfer;
    public final VisionSubsystem vision;

    Constants.ShooterConfig pollenConfig;

    public Subsystems(HardwareMap hardwareMap, Pose startingPose, int pipeline){

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hubs : allHubs){
            hubs.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        pollenConfig = new Constants.ShooterConfig(
                Constants.EHM3,
                Constants.POLLEN_PIDF
        );

        drive = new DriveSubsystem(hardwareMap, startingPose);
        intake = new IntakeSubsystem(hardwareMap);
        pollen = new ShooterSubsystem(hardwareMap, pollenConfig);
        transfer = new TransferSubsystem(hardwareMap);
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
        //?Feed the distance to the shooters
        pollen.setDistance(drive.getDistance());

        //? Let the transfer know how many balls are counted
        transfer.setBallCount(intake.ballCount());
    }

}
