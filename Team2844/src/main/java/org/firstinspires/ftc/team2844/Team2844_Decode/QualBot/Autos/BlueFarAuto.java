package org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Hardwares.LimelightHardware;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Hardwares.ShooterHardware;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.RoadrunnerQuickstart.MecanumDrive;

@Autonomous(name = "Blue far Goal")
public class BlueFarAuto extends LinearOpMode {

    long BUFFER_TIME = 1200;
    Pose2d estimate;



    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPos = new Pose2d(-63.25,8.75,0.0);
        ShooterHardware shooterHardware = new ShooterHardware(this);
        LimelightHardware limelightHardware = new LimelightHardware(this);
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, initialPos);
        limelightHardware.innit(0);
        mecanumDrive.updatePoseEstimate();
        estimate = mecanumDrive.localizer.getPose();
        Servo gobildaLight = hardwareMap.get(Servo.class, "gobildaLight");
        gobildaLight.setPosition(0.611);

        waitForStart();
        if (isStopRequested()) return;

        TrajectoryActionBuilder moveToShoot1 = mecanumDrive.actionBuilder(initialPos)
                .lineToX(-60)
                .turnTo(Math.toRadians(25));

        TrajectoryActionBuilder moveOut = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(-60, 8.75), Math.toRadians(25)))
                .strafeTo(new Vector2d(-60, 34))
                .turnTo(Math.toRadians(-90));




        //start of moving
        Actions.runBlocking(moveToShoot1.build());
        if(limelightHardware.getTx() != -999){
            TrajectoryActionBuilder rotateShoot1 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(-60, 8.75), Math.toRadians(25)))
                    .turn(Math.toRadians(-limelightHardware.getTx()));
            Actions.runBlocking(rotateShoot1.build());
        }

        shoot(shooterHardware, limelightHardware);

        Actions.runBlocking(moveOut.build());



    }

    private void shoot(ShooterHardware shooterHardware, LimelightHardware limelightHardware){
        //int count = 0;
        while (shooterHardware.oneBall() && opModeIsActive()) {
            double shooterVelocity = shooterHardware.getShootSpeed(limelightHardware.getBotDis());
            if(limelightHardware.getTx() != -999){
                shooterVelocity = shooterHardware.getShootSpeed(limelightHardware.getBotDis());
                shooterHardware.aimHood(shooterHardware.getHoodAim(limelightHardware.getBotDis()));
            } else {
                shooterVelocity = shooterHardware.getShootSpeed(120.0);
                shooterHardware.aimHood(shooterHardware.getHoodAim(120.0));
            }
            shooterHardware.setShootVelocity(shooterVelocity);
            if(shooterHardware.withinVel(shooterVelocity)) {
                shooterHardware.feed();
            } else if(shooterHardware.belowVel(shooterVelocity)) {
                shooterHardware.stopFeed();
                //count = 0;
            }
            //count++;
        }
        shooterHardware.feed();
        sleep(BUFFER_TIME);
        shooterHardware.setShootVelocity(0.0);
        shooterHardware.aimHood(0.0);
        shooterHardware.stopFeed();
    }
}
