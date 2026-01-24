package org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Autos;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Hardwares.LimelightHardware;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Hardwares.ShooterHardware;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.RoadrunnerQuickstart.MecanumDrive;

import java.util.Arrays;

@Autonomous(name = "Red Near Goal")
public class RedCloseAuto extends LinearOpMode {

    long BUFFER_TIME = 1200;
    Pose2d estimate;



    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPos = new Pose2d(55,-45,-0.785398);
        ShooterHardware shooterHardware = new ShooterHardware(this);
        LimelightHardware limelightHardware = new LimelightHardware(this);
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, initialPos);
        limelightHardware.innit(1);
        mecanumDrive.updatePoseEstimate();
        estimate = mecanumDrive.localizer.getPose();
        boolean skip = false;

        boolean highVoltage = hardwareMap.voltageSensor.get("Control Hub").getVoltage() > 13.0;

        VelConstraint baseSpeed = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(60),
                new AngularVelConstraint((6*Math.PI))
        ));

        VelConstraint lastSpeed = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(50),
                new AngularVelConstraint((6*Math.PI))
        ));

        TurnConstraints shakeConstraints = new TurnConstraints((Math.PI), -(3*Math.PI), (3*Math.PI));

        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-55.0, 55.0);
        AccelConstraint backUpAccelConstraint = new ProfileAccelConstraint(-25.0, 25.0);
        AccelConstraint longAccelConstraint = new ProfileAccelConstraint(-35.0, 35.0);


        waitForStart();
        if (isStopRequested()) return;

        TrajectoryActionBuilder moveToShoot1 = mecanumDrive.actionBuilder(initialPos)
                .lineToYSplineHeading(-25, -(0.785398+Math.toRadians(5)), baseSpeed, baseAccelConstraint);

        TrajectoryActionBuilder pickupBalls1 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(40, -25), -0.785398))
                .splineToLinearHeading(new Pose2d(12, -25, -Math.PI/2), -Math.PI/2, baseSpeed, baseAccelConstraint)
                .setReversed(true)
                .strafeToConstantHeading(new Vector2d(12, -60), baseSpeed, baseAccelConstraint)
                .lineToYConstantHeading(-45, lastSpeed, backUpAccelConstraint);

        TrajectoryActionBuilder backUpToShoot1 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(12, -45), -Math.PI/2))
                .splineToSplineHeading(new Pose2d(24, -24, -Math.PI/3.5), -Math.PI/2, lastSpeed, longAccelConstraint);

        TrajectoryActionBuilder backUp1 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(12, -45), -Math.PI/2))
                .turn(Math.toRadians(-15), shakeConstraints)
                .turn(Math.toRadians(30), shakeConstraints);


        TrajectoryActionBuilder moveToShoot2 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(12, -45), -((Math.PI/2)+Math.toRadians(15))))
                .splineToLinearHeading(new Pose2d(24, -24, -Math.PI/3.5), -2*Math.PI/3, lastSpeed, backUpAccelConstraint);

        TrajectoryActionBuilder pickupBalls2 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(24, -24), -Math.PI/3.5))
                .splineToLinearHeading(new Pose2d(-12, -24, -Math.PI/2), -(Math.PI)/2, baseSpeed, baseAccelConstraint)
                .setReversed(true)
                .strafeToConstantHeading(new Vector2d(-12, -66), baseSpeed, baseAccelConstraint)
                .lineToYConstantHeading(-50, lastSpeed, backUpAccelConstraint);;

        TrajectoryActionBuilder backUpToShoot2 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(-12, -45), -Math.PI/2))
                .lineToY(-35, lastSpeed)
                .strafeToLinearHeading(new Vector2d(36, -20), -(Math.PI/3.5), lastSpeed, new ProfileAccelConstraint(-35, 35));

        TrajectoryActionBuilder backUp2 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(-12, -45), -Math.PI/2))
                .lineToY(-40, baseSpeed, backUpAccelConstraint)
                .turn(Math.toRadians(-15), shakeConstraints)
                .turn(Math.toRadians(30), shakeConstraints);

        TrajectoryActionBuilder moveToShoot3 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(-12, -45), -((Math.PI/2)+Math.toRadians(15))))
                .splineToSplineHeading(new Pose2d(36, -20, -(Math.PI/3.5)), 0, lastSpeed, new ProfileAccelConstraint(-30, 30));


        //start of moving
        shooterHardware.setShootPower(0.2);
        Actions.runBlocking(moveToShoot1.build());

        shoot(shooterHardware, limelightHardware);

        shooterHardware.closeServo();
        if(highVoltage){
            shooterHardware.intake(.9);
            shooterHardware.setShootPower(-0.35);
        } else {
            shooterHardware.intake(1);
            shooterHardware.setShootPower(-.25);
        }
        Actions.runBlocking(pickupBalls1.build());


        if(shooterHardware.threeBall()){
            shooterHardware.intake(0);
            shooterHardware.setShootPower(0.2);
            Actions.runBlocking(backUpToShoot1.build());
        } else {
            skip = true;
            Actions.runBlocking(backUp1.build());
            shooterHardware.stopFeed();
            highVoltage = hardwareMap.voltageSensor.get("Control Hub").getVoltage() > 13.0;
            shooterHardware.setShootPower(0.2);
            Actions.runBlocking(moveToShoot2.build());
        }

        shooterHardware.stopFeed();

        if(limelightHardware.getTx() != -999){
            TrajectoryActionBuilder rotateShoot2 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(24, -24), -Math.PI/3.5))
                    .turn(Math.toRadians(-limelightHardware.getTx()-6));
            Actions.runBlocking(rotateShoot2.build());
        }

        shoot(shooterHardware, limelightHardware);

        shooterHardware.closeServo();
        if(highVoltage){
            shooterHardware.intake(.8);
            shooterHardware.setShootPower(-0.35);
        } else {
            shooterHardware.intake(1);
            shooterHardware.setShootPower(-.25);
        }

        Actions.runBlocking(pickupBalls2.build());

        if(shooterHardware.threeBall() || skip){
            shooterHardware.setShootPower(0.2);
            Actions.runBlocking(backUpToShoot2.build());
            shooterHardware.intake(0);
        } else {
            Actions.runBlocking(backUp2.build());

            shooterHardware.stopFeed();
            highVoltage = hardwareMap.voltageSensor.get("Control Hub").getVoltage() > 13.0;
            shooterHardware.setShootPower(0.2);

            Actions.runBlocking(moveToShoot3.build());
        }

        shooterHardware.stopFeed();
        shooterHardware.setShootPower(0.2);

        if(limelightHardware.getTx() != -999){
            TrajectoryActionBuilder rotateShoot3 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(36, -20), -(Math.PI/3.5)))
                    .turn(Math.toRadians(-limelightHardware.getTx()));
            Actions.runBlocking(rotateShoot3.build());
        }
        shoot(shooterHardware, limelightHardware);

    }

    private void shoot(ShooterHardware shooterHardware, LimelightHardware limelightHardware){
        //int count = 0;
        double shooterVelocity = shooterHardware.getShootSpeed(limelightHardware.getBotDis());

        while (shooterHardware.oneBall() && opModeIsActive()) {
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
        shooterHardware.setShootPower(shooterHardware.getShootPowerLINREG(shooterVelocity));
        shooterHardware.feed();
        sleep(BUFFER_TIME);
        shooterHardware.setShootVelocity(0.0);
        shooterHardware.aimHood(0.0);
        shooterHardware.stopFeed();
    }
}
