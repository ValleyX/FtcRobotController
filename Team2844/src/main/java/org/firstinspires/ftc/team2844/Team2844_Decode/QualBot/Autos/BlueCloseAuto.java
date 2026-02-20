package org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Autos;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Autos.Actions.IntakeAction;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Autos.Actions.ShootAction;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Autos.Actions.StopIntakeAction;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Hardwares.LimelightHardware;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Hardwares.ShooterHardware;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.RoadrunnerQuickstart.MecanumDrive;

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
        boolean skip = false;
        Servo gobildaLight = hardwareMap.get(Servo.class, "gobildaLight");
        gobildaLight.setPosition(0.611);

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
                .lineToYSplineHeading(25, (0.785398+Math.toRadians(5)), baseSpeed, baseAccelConstraint);

        TrajectoryActionBuilder pickupBalls1 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(40, 25), 0.785398))
                .splineToLinearHeading(new Pose2d(12, 25, Math.PI/2), Math.PI/2, baseSpeed, baseAccelConstraint)
                .setReversed(true)
                .strafeToConstantHeading(new Vector2d(12, 60), baseSpeed, baseAccelConstraint)
                .lineToYConstantHeading(45, lastSpeed, backUpAccelConstraint);

        TrajectoryActionBuilder backUpToShoot1 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(12, 45), Math.PI/2))
                .splineToSplineHeading(new Pose2d(24, 24, Math.PI/3.5), Math.PI/2, lastSpeed, longAccelConstraint);

        TrajectoryActionBuilder backUp1 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(12, 45), Math.PI/2))
                .turn(Math.toRadians(-15), shakeConstraints)
                .turn(Math.toRadians(30), shakeConstraints);


        TrajectoryActionBuilder moveToShoot2 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(12, 45), (Math.PI/2)+Math.toRadians(15)))
                .splineToLinearHeading(new Pose2d(24, 24, Math.PI/3.5), 2*Math.PI/3, lastSpeed, backUpAccelConstraint);

        TrajectoryActionBuilder pickupBalls2 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(24, 24), Math.PI/3.5))
                .splineToLinearHeading(new Pose2d(-12, 18, Math.PI/2), (Math.PI)/2, baseSpeed, baseAccelConstraint)
                .setReversed(true)
                .strafeToConstantHeading(new Vector2d(-12, 66), baseSpeed, baseAccelConstraint)
                .lineToYConstantHeading(50, lastSpeed, backUpAccelConstraint);;

        TrajectoryActionBuilder backUpToShoot2 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(-12, 45), Math.PI/2))
                .lineToY(35, lastSpeed)
                .strafeToLinearHeading(new Vector2d(24, 24), (Math.toRadians(45)), lastSpeed, new ProfileAccelConstraint(-35, 35));

        TrajectoryActionBuilder backUp2 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(-12, 45), Math.PI/2))
                .lineToY(43, baseSpeed, backUpAccelConstraint)
                .turn(Math.toRadians(-15), shakeConstraints)
                .turn(Math.toRadians(30), shakeConstraints);

        TrajectoryActionBuilder moveToShoot3 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(-12, 43), (Math.PI/2)))
                .splineToSplineHeading(new Pose2d(24, 24, Math.toRadians(45)), 0, lastSpeed, new ProfileAccelConstraint(-25, 25));

        TrajectoryActionBuilder moveOut = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(24, 24), Math.toRadians(55)))
                .setTangent(Math.toRadians(180.0))
                .splineToLinearHeading(new Pose2d(0, 48, Math.toRadians(-90.0)), Math.toRadians(180.0));


        //start of moving
        shooterHardware.setShootVelocity(20.0);
        Actions.runBlocking(new SequentialAction(
                moveToShoot1.build(),
                new ShootAction(shooterHardware, limelightHardware, this),
                new IntakeAction(shooterHardware),
                pickupBalls1.build()
        ));


        if(shooterHardware.threeBall()){
            shooterHardware.intake(0);
            shooterHardware.setShootVelocity(20.0);
            Actions.runBlocking(backUpToShoot1.build());
        } else {
            skip = true;
            Actions.runBlocking(new SequentialAction(
                    backUp1.build(),
                    moveToShoot2.build()
            ));
        }

        shooterHardware.stopFeed();

        if(limelightHardware.getTx() != -999){
            TrajectoryActionBuilder rotateShoot2 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(24, 24), Math.PI/3.5))
                    .turn(Math.toRadians(-limelightHardware.getTx()+6));
            Actions.runBlocking(rotateShoot2.build());
        }

        Actions.runBlocking(new SequentialAction(
                new ShootAction(shooterHardware, limelightHardware, this),
                new IntakeAction(shooterHardware),
                pickupBalls2.build()
        ));


        if(shooterHardware.threeBall() || skip){
            shooterHardware.setShootVelocity(20.0);
            Actions.runBlocking(new SequentialAction(
                    backUpToShoot2.build(),
                    new StopIntakeAction(shooterHardware)
            ));
        } else {
            Actions.runBlocking(new SequentialAction(
                    backUp2.build(),
                    moveToShoot3.build()
            ));
        }

        shooterHardware.stopFeed();
        shooterHardware.setShootVelocity(20.0);

        if(limelightHardware.getTx() != -999){
            TrajectoryActionBuilder rotateShoot3 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(24, 24), Math.toRadians(55)))
                    .turn(Math.toRadians(-limelightHardware.getTx()));
            Actions.runBlocking(rotateShoot3.build());
        }

        Actions.runBlocking( new SequentialAction(
                new ShootAction(shooterHardware, limelightHardware, this)//,
                //moveOut.build()
        ));

    }

    private void shoot(ShooterHardware shooterHardware, LimelightHardware limelightHardware){
        //int count = 0;
        double shooterVelocity = shooterHardware.getShootSpeed(limelightHardware.getBotDis());

        //move ghetto arm away from ball to shoot
        shooterHardware.stopBallRelease();//jae


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
        shooterHardware.setShootVelocity(20.0);
        shooterHardware.aimHood(0.0);
        shooterHardware.stopFeed();

        //move ghetto arm back on top of ball to shoot
        shooterHardware.stopBallHold();//jae
    }
}
