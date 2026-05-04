package org.firstinspires.ftc.team12841;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.team12841.pedroPathing.Constants;
import org.firstinspires.ftc.team12841.subsystems.*;

public class RobotContainer {

    public final DriveSubsystem drive;
    public final ShooterSubsystem shooter;
    public final IntakeSubsystem intake;
    public final VisionSubsystem vision;
    public final LightSubsystem lights;

    private final Follower follower;

    public RobotContainer(OpMode opMode) {
        HardwareMap hw = opMode.hardwareMap;

        follower = Constants.createFollower(hw);

        drive   = new DriveSubsystem(hw, follower);
        vision  = new VisionSubsystem(hw);
        shooter = new ShooterSubsystem(hw, vision);
        intake  = new IntakeSubsystem(hw);
        lights  = new LightSubsystem(hw);
    }

    public Follower getFollower() {
        return follower;
    }
}