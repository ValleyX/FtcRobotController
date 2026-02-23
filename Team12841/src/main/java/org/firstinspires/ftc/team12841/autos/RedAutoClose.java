package org.firstinspires.ftc.team12841.autos;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Close", group = "AUTO")
public class RedAutoClose extends AutoBase {

    @Override
    public void init() {
        super.init();
        super.pipeline = 2;

        Pose startPose = new Pose(0, 0, 0);
        Pose shootPreloadPose = new Pose(-48.24, -6, -0.0621);

        Pose alignIntake1 = new Pose(-41.878, -11.635, -0.7688);
        Pose intake1Pose = new Pose(-24.81, -29.279, -0.7956);
        Pose shoot1Pose = new Pose(-45.53, -11, 0.0724);

        Pose alignIntake2 = new Pose(-53.6084, -20.4814, -0.823);
        Pose intake2Pose = new Pose(-32.1654, -48.2741, -0.823);
        Pose shoot2Pose = new Pose(-47.7855, -5.4689, -0.088);

        Pose parkPose = new Pose(-28.5896, 11.6066, -0.846);

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