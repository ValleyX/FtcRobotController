package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vcs.valleylib.ftc.hardware.FtcSubsystem;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.RobotConstants;

import java.util.function.DoubleSupplier;

/**
 * Limelight 3A wrapper.
 *
 * <p>Unlike the old {@code LimelightHardware}, which re-polled the camera inside
 * every getter, this pulls one result per scheduler loop in {@link #periodic()}.
 * Every reader in a given loop therefore sees the same frame, and a shot solution
 * can't be computed from a distance and a tx that came from different frames.
 *
 * <p>Getters return {@link RobotConstants#NO_TARGET} when there is no valid target.
 */
public class VisionSubsystem extends FtcSubsystem {

    private final Limelight3A limelight;

    /** Feeds robot heading to the Limelight so MegaTag2 can resolve pose. */
    private final DoubleSupplier headingDegrees;

    private LLResult latestResult;
    private int pipeline;

    public VisionSubsystem(HardwareMap hardwareMap, DoubleSupplier headingDegrees) {
        super(hardwareMap);
        this.headingDegrees = headingDegrees;
        limelight = hardwareMap.get(Limelight3A.class, RobotConstants.LIMELIGHT);
    }

    /** Selects a pipeline and starts streaming. Call during opmode init. */
    public void start(int pipeline) {
        this.pipeline = pipeline;
        limelight.pipelineSwitch(pipeline);
        limelight.start();
    }

    public int getPipeline() {
        return pipeline;
    }

    @Override
    public void periodic() {
        limelight.updateRobotOrientation(headingDegrees.getAsDouble());
        latestResult = limelight.getLatestResult();
    }

    private boolean hasTarget() {
        return latestResult != null && latestResult.isValid();
    }

    /** True when the current frame has a usable target. */
    public boolean isTargetVisible() {
        return hasTarget();
    }

    /** Horizontal offset to the target in degrees; positive means target is right. */
    public double getTx() {
        return hasTarget() ? latestResult.getTx() : RobotConstants.NO_TARGET;
    }

    /** Vertical offset to the target in degrees. */
    public double getTy() {
        return hasTarget() ? latestResult.getTy() : RobotConstants.NO_TARGET;
    }

    /** Fraction of the image the target fills. */
    public double getTargetArea() {
        return hasTarget() ? latestResult.getTa() : RobotConstants.NO_TARGET;
    }

    /** Distance to the tag in inches — the input to every shot regression. */
    public double getDistanceInches() {
        return hasTarget() ? latestResult.getBotposeAvgDist() * 39.37 : RobotConstants.NO_TARGET;
    }

    public double getBotX() {
        if (!hasTarget()) {
            return RobotConstants.NO_TARGET;
        }
        Pose3D botPose = latestResult.getBotpose();
        return botPose.getPosition().x * 254;
    }

    public double getBotY() {
        if (!hasTarget()) {
            return RobotConstants.NO_TARGET;
        }
        Pose3D botPose = latestResult.getBotpose();
        return botPose.getPosition().y * 254;
    }

    public double getBotZ() {
        if (!hasTarget()) {
            return RobotConstants.NO_TARGET;
        }
        Pose3D botPose = latestResult.getBotpose();
        return botPose.getPosition().z * 254;
    }
}
