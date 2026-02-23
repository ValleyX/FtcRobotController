package org.firstinspires.ftc.team12841.autos;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Far", group = "AUTO")
public class BlueAutoFar extends AutoBase {

    @Override
    public void init() {
        super.init();
        super.pipeline = 1;

        Pose startPose = new Pose(0, 0, 0);
        Pose shootPreloadPose = new Pose(9.6771, -3.4543, 0.3651);

        Pose alignIntake1 = new Pose(31.0451, 5.3002, 1.6138);
        Pose intake1Pose = new Pose(35.7305, 44.3417, 1.6798);
        Pose shoot1Pose = new Pose(9, 2.5734, 0.3018);

        Pose alignIntake2 = new Pose(25, 35, 1.6033);
        Pose intake2Pose = new Pose(9, 42, 1.6033);
        Pose shoot2Pose = new Pose(15, 2, 0.3651);

        Pose parkPose = new Pose(30.2975, 0.0434, 1.6033);

        super.follower.setStartingPose(startPose);

        super.START_TO_SHOOT = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPreloadPose))
                .setLinearHeadingInterpolation(
                        startPose.getHeading(),
                        shootPreloadPose.getHeading()
                )
                .build();

        super.SHOOT_TO_ALIGN1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPreloadPose, alignIntake1))
                .setLinearHeadingInterpolation(
                        shootPreloadPose.getHeading(),
                        alignIntake1.getHeading()
                )
                .build();

        super.ALIGN1_TO_INTAKE1 = follower.pathBuilder()
                .addPath(new BezierLine(alignIntake1, intake1Pose))
                .setLinearHeadingInterpolation(
                        alignIntake1.getHeading(),
                        intake1Pose.getHeading()
                )
                .build();

        super.INTAKE1_TO_SHOOT = follower.pathBuilder()
                .addPath(new BezierLine(intake1Pose, shoot1Pose))
                .setLinearHeadingInterpolation(
                        intake1Pose.getHeading(),
                        shoot1Pose.getHeading()
                )
                .build();

        super.SHOOT_TO_ALIGN2 = follower.pathBuilder()
                .addPath(new BezierLine(shoot1Pose, alignIntake2))
                .setLinearHeadingInterpolation(
                        shoot1Pose.getHeading(),
                        alignIntake2.getHeading()
                )
                .build();

        super.ALIGN2_TO_INTAKE2 = follower.pathBuilder()
                .addPath(new BezierLine(alignIntake2, intake2Pose))
                .setLinearHeadingInterpolation(
                        alignIntake2.getHeading(),
                        intake2Pose.getHeading()
                )
                .build();

        super.INTAKE2_TO_SHOOT = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, shoot2Pose))
                .setLinearHeadingInterpolation(
                        intake2Pose.getHeading(),
                        shoot2Pose.getHeading()
                )
                .build();

        super.SHOOT_AFTER_INTAKE2 = follower.pathBuilder()
                .addPath(new BezierLine(shoot2Pose, shoot1Pose))
                .setLinearHeadingInterpolation(
                        shoot2Pose.getHeading(),
                        shoot1Pose.getHeading()
                )
                .build();

        super.SHOOT_FAR_TO_PARK = follower.pathBuilder()
                .addPath(new BezierLine(shoot2Pose, parkPose))
                .setLinearHeadingInterpolation(
                        shoot2Pose.getHeading(),
                        parkPose.getHeading()
                )
                .build();
    }
}