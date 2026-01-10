package org.firstinspires.ftc.team2844.Team2844_Decode.Autos;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team2844.Team2844_Decode.Hardwares.LimelightHardware;
import org.firstinspires.ftc.team2844.Team2844_Decode.Hardwares.ShooterHardware;
import org.firstinspires.ftc.team2844.Team2844_Decode.RoadrunnerStuff.RoadrunnerQuickstart.MecanumDrive;

import java.util.Arrays;

@Autonomous(name = "Blue Near Goal")
public class BlueCloseAuto extends LinearOpMode {

    long BUFFER_TIME = 1200;
    Pose2d estimate;



    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPos = new Pose2d(55,45,0.785398);
        ShooterHardware shooterHardware = new ShooterHardware(this);
        LimelightHardware limelightHardware = new LimelightHardware(this);
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, initialPos);
        limelightHardware.innit(0);
        mecanumDrive.updatePoseEstimate();
        estimate = mecanumDrive.localizer.getPose();

        VelConstraint baseSpeed = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(250),
                new AngularVelConstraint((4*Math.PI))
        ));

        VelConstraint basePickupSpeed = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(200),
                new AngularVelConstraint((4*Math.PI))
        ));

        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-50.0, 50.0);

        waitForStart();
        if (isStopRequested()) return;

        TrajectoryActionBuilder moveToShoot1 = mecanumDrive.actionBuilder(initialPos)
                .lineToYConstantHeading(25, baseSpeed, baseAccelConstraint);

        TrajectoryActionBuilder pickupBalls1 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(40, 25), 0.785398))
                .splineToLinearHeading(new Pose2d(8.5, 25, Math.PI/2), Math.PI/2, baseSpeed, baseAccelConstraint)
                .strafeToConstantHeading(new Vector2d(8.5, 60), basePickupSpeed, baseAccelConstraint);

        TrajectoryActionBuilder moveToShoot2 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(8.5, 60), (Math.PI/2)))
                .splineToLinearHeading(new Pose2d(24, 24, 0.785398), (3*Math.PI)/2, baseSpeed, baseAccelConstraint);

        TrajectoryActionBuilder pickupBalls2 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(24, 24), 0.785398))
                .splineToLinearHeading(new Pose2d(-15, 24, Math.PI/2), (Math.PI)/2, baseSpeed, baseAccelConstraint)
                .strafeToConstantHeading(new Vector2d(-15, 66), basePickupSpeed, baseAccelConstraint);

        TrajectoryActionBuilder moveToShoot3 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(-14.5, 66), Math.PI/2))
                .lineToY(40, baseSpeed, baseAccelConstraint)
                .splineToLinearHeading(new Pose2d(36, 12, 0.785398), (Math.PI)/2, baseSpeed, baseAccelConstraint);


        //start of moving
        Actions.runBlocking(moveToShoot1.build());

        shoot(shooterHardware, limelightHardware);

        shooterHardware.intake(1);
        shooterHardware.setShootPower(-0.25);
        Actions.runBlocking(pickupBalls1.build());


        if(shooterHardware.threeBall()){
            shooterHardware.stopFeed();
            shooterHardware.setShootPower(0.1);
        }

        Actions.runBlocking(moveToShoot2.build());
        shooterHardware.stopFeed();
        if(limelightHardware.getTx() != -999){
            TrajectoryActionBuilder rotateShoot2 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(24, 24), 0.785398))
                    .turn(Math.toRadians(-limelightHardware.getTx() + 5));
            Actions.runBlocking(rotateShoot2.build());
        }

        shoot(shooterHardware, limelightHardware);

        shooterHardware.intake(1);
        shooterHardware.setShootPower(-0.25);
        Actions.runBlocking(pickupBalls2.build());

        if(shooterHardware.threeBall()){
            shooterHardware.stopFeed();
            shooterHardware.setShootPower(0.1);
        }

        Actions.runBlocking(moveToShoot3.build());
        shooterHardware.stopFeed();
        if(limelightHardware.getTx() != -999){
            TrajectoryActionBuilder rotateShoot3 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(36, 12), 0.785398))
                    .turn(Math.toRadians(-limelightHardware.getTx()));
            Actions.runBlocking(rotateShoot3.build());
        }
        shoot(shooterHardware, limelightHardware);

    }

    private void shoot(ShooterHardware shooterHardware, LimelightHardware limelightHardware){
        //int count = 0;
        while (shooterHardware.oneBall() && opModeIsActive()) {
            double shooterVelocity = shooterHardware.getShootSpeed(limelightHardware.getBotDis());
            shooterHardware.setShootVelocity(shooterVelocity);
            shooterHardware.aimHood(shooterHardware.getHoodAim(limelightHardware.getBotDis()));
            if(shooterHardware.withinVel(shooterVelocity)) {
                shooterHardware.setShootPower(shooterHardware.getShootPowerLINREG(shooterVelocity));
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
