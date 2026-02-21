package org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Autos.Actions.FarShootAction;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Autos.Actions.IntakeAction;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Autos.Actions.StopIntakeAction;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Hardwares.LimelightHardware;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Hardwares.QualConstants;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Hardwares.ShooterHardware;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.RoadrunnerQuickstart.MecanumDrive;

@Autonomous(name = "Red far Goal 6 Ball")
public class BlueFarAutoSixBall extends LinearOpMode {


    Pose2d estimate;



    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPos = new Pose2d(-63.5,17,0.0);
        ShooterHardware shooterHardware = new ShooterHardware(this);
        LimelightHardware limelightHardware = new LimelightHardware(this);
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, initialPos);
        limelightHardware.innit(1);
        mecanumDrive.updatePoseEstimate();
        estimate = mecanumDrive.localizer.getPose();
        Servo gobildaLight = hardwareMap.get(Servo.class, "timerLight");
        gobildaLight.setPosition(0.611);

        waitForStart();
        if (isStopRequested()) return;

        // 3 balls
        TrajectoryActionBuilder moveToShoot1 = mecanumDrive.actionBuilder(initialPos)
                .lineToX(-60)
                .turnTo(Math.toRadians(25));


        //6 balls
        /*
        TrajectoryActionBuilder moveToPickup1 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(-60, -17), Math.toRadians(-25)))
                .setTangent(Math.toRadians(-45.0))
                .splineToLinearHeading(new Pose2d(new Vector2d(-41,-24), Math.toRadians(-90.0)), Math.toRadians(-130.0));

        TrajectoryActionBuilder grab1 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(-41, -24), Math.toRadians(-90.0)))
                .lineToY(-72+9);

        TrajectoryActionBuilder moveToShoot2 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(-41,-72+9), Math.toRadians(-125.0)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(new Vector2d(-60, -12), Math.toRadians(-25.0)), Math.toRadians(0.0));

         */



        // 9 balls
        TrajectoryActionBuilder moveToPickup2 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(-60, 17), Math.toRadians(25.0)))
                .setTangent(45.0)
                .splineToLinearHeading(new Pose2d(new Vector2d(-55, 48), Math.toRadians(130.0)), Math.toRadians(90.0));

        TrajectoryActionBuilder grab2 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(-55, 48), Math.toRadians(130.0)))
                .setTangent(45)
                .splineToConstantHeading(new Vector2d(-72, 72), Math.toRadians(0.0));

        TrajectoryActionBuilder moveToShoot3 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(-72+8, 72-8), Math.toRadians(90.0)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(new Vector2d(-60, 12), Math.toRadians(25.0)), Math.toRadians(0.0));


        TrajectoryActionBuilder moveOut = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(-60, 12), Math.toRadians(25.0)))
                .strafeToLinearHeading(new Vector2d(-60, 36), Math.toRadians(-90.0));




        shooterHardware.setShootVelocity(100.0);
        //start of moving
        Actions.runBlocking(moveToShoot1.build());

        if(limelightHardware.getTx() != -999){
            TrajectoryActionBuilder rotateShoot1 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(-60, 17), Math.toRadians(25)))
                    .turn(Math.toRadians(-limelightHardware.getTx()));
            Actions.runBlocking(rotateShoot1.build());
        }

        /*Actions.runBlocking(new SequentialAction(
                new FarShootAction(shooterHardware, limelightHardware, this),
                new IntakeAction(shooterHardware),
                moveToPickup1.build(),
                grab1.build(),
                moveToShoot2.build()
        ));

        if(limelightHardware.getTx() != -999){
            TrajectoryActionBuilder rotateShoot1 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(-60, -12), Math.toRadians(-25)))
                    .turn(Math.toRadians(-limelightHardware.getTx()));
            Actions.runBlocking(rotateShoot1.build());
        }*/

        Actions.runBlocking( new SequentialAction(
                new StopIntakeAction(shooterHardware),
                new FarShootAction(shooterHardware, limelightHardware, this),
                new IntakeAction(shooterHardware),
                moveToPickup2.build(),
                grab2.build(),
                moveToShoot3.build()
        ));

        if(limelightHardware.getTx() != -999){
            TrajectoryActionBuilder rotateShoot1 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(-60, 12), Math.toRadians(25)))
                    .turn(Math.toRadians(-limelightHardware.getTx()));
            Actions.runBlocking(rotateShoot1.build());
        }

        Actions.runBlocking(new SequentialAction(
                new FarShootAction(shooterHardware, limelightHardware, this),
                moveOut.build()
        ));


    }

    private void shoot(ShooterHardware shooterHardware, LimelightHardware limelightHardware){
        //int count = 0;
        //move ghetto arm away from ball to shoot
        shooterHardware.stopBallRelease();//jae

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
        sleep(QualConstants.BUFFER_TIME);
        shooterHardware.setShootVelocity(20.0);
        shooterHardware.aimHood(0.0);
        shooterHardware.stopFeed();

        //move ghetto arm back on top of ball to shoot
        shooterHardware.stopBallHold();//jae
    }
}
