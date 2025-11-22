package org.firstinspires.ftc.team2844.Team2844_Decode.Autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team2844.Team2844_Decode.Hardwares.LimelightHardware;
import org.firstinspires.ftc.team2844.Team2844_Decode.Hardwares.RobotHardware;
import org.firstinspires.ftc.team2844.Team2844_Decode.Hardwares.ShooterHardware;
import org.firstinspires.ftc.team2844.Team2844_Decode.RoadrunnerStuff.RoadrunnerQuickstart.MecanumDrive;

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

        waitForStart();
        if (isStopRequested()) return;

        TrajectoryActionBuilder moveToShoot1 = mecanumDrive.actionBuilder(initialPos)
                .lineToYConstantHeading(25);

        TrajectoryActionBuilder pickupBalls1 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(40, 25), 0.785398))
                .turnTo(Math.PI/2)
                .strafeToConstantHeading(new Vector2d(8.5, 25))
                .strafeToConstantHeading(new Vector2d(8.5, 60));

        TrajectoryActionBuilder moveToShoot2 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(8.5, 60), (Math.PI/2)))
                .strafeToLinearHeading(new Vector2d(24, 24), 0.785398);

        TrajectoryActionBuilder pickupBalls2 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(24, 24), 0.785398))
                .turnTo(Math.PI/2)
                .strafeToConstantHeading(new Vector2d(-14.5, 24))
                .strafeToConstantHeading(new Vector2d(-14.5, 66));

        TrajectoryActionBuilder moveToShoot3 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(-14.5, 66), Math.PI/2))
                .lineToY(40)
                .strafeToLinearHeading(new Vector2d(36, 12), 0.785398);


        //start of moving
        Actions.runBlocking(moveToShoot1.build());
        if(limelightHardware.getTx() != -999){
            TrajectoryActionBuilder rotateShoot1 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(40, 25), 0.785398))
                    .turn(Math.toRadians(-limelightHardware.getTx()));
            Actions.runBlocking(rotateShoot1.build());
        }

        shoot(shooterHardware, limelightHardware);

        shooterHardware.intake(1.0);
        Actions.runBlocking(pickupBalls1.build());
        shooterHardware.stopFeed();


        shooterHardware.shoot(0.25);
        Actions.runBlocking(moveToShoot2.build());
        if(limelightHardware.getTx() != -999){
            TrajectoryActionBuilder rotateShoot2 = mecanumDrive.actionBuilder(new Pose2d(new Vector2d(24, 24), 0.785398))
                    .turn(Math.toRadians(-limelightHardware.getTx()));
            Actions.runBlocking(rotateShoot2.build());
        }

        shoot(shooterHardware, limelightHardware);

        shooterHardware.intake(1.0);
        Actions.runBlocking(pickupBalls2.build());
        shooterHardware.stopFeed();


        shooterHardware.shoot(0.25);
        Actions.runBlocking(moveToShoot3.build());
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
