package org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Autos.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Hardwares.LimelightHardware;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Hardwares.QualConstants;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBot.Hardwares.ShooterHardware;

import java.util.concurrent.TimeUnit;


public class FarShootAction implements Action {

    ShooterHardware shooterHardware;
    LimelightHardware limelightHardware;
    LinearOpMode opMode;
    boolean init;
    ElapsedTime time;

    public FarShootAction(ShooterHardware shooterHardware, LimelightHardware limelightHardware, LinearOpMode opMode) {
        this.shooterHardware = shooterHardware;
        this.limelightHardware = limelightHardware;
        this.opMode = opMode;
        time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        time.reset();
        init = true;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        //int count = 0;
        //move ghetto arm away from ball to shoot
        shooterHardware.stopBallRelease();//jae
//        if(init) {
//            shooterHardware.intake(0.0);
//            init = false;
//        }

        if (shooterHardware.oneBall() && opMode.opModeIsActive()) {
            double shooterVelocity = shooterHardware.getShootSpeed(limelightHardware.getBotDis());
            if(limelightHardware.getTx() != -999){
                shooterVelocity = shooterHardware.getShootSpeed(limelightHardware.getBotDis());
                shooterHardware.aimHood(shooterHardware.getHoodAim(limelightHardware.getBotDis()));
            } else {
                shooterVelocity = shooterHardware.getShootSpeed(128.0);
                shooterHardware.aimHood(shooterHardware.getHoodAim(128.0));
            }
            shooterHardware.setShootVelocity(shooterVelocity);
            if(shooterHardware.withinVel(shooterVelocity)) {
                time.reset();
                shooterHardware.feed();
            } else if(shooterHardware.belowVel(shooterVelocity)) {
                shooterHardware.stopFeed();
                //count = 0;
            }
            //count++;
            return true;
        } else {
            shooterHardware.feed();
            opMode.sleep(QualConstants.BUFFER_TIME);
            shooterHardware.setShootVelocity(100.0);
            shooterHardware.aimHood(0.0);
            shooterHardware.stopFeed();
            //move ghetto arm back on top of ball to shoot
            shooterHardware.stopBallHold();//jae

            return false;
        }
    }
}
