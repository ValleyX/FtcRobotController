package org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.subsystems.DriveSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.subsystems.LightSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.team2844.Team2844_Decode.QualBotCommand.subsystems.VisionSubsystem;

/**
 * Every subsystem on the Qual Bot, built once and shared by every opmode.
 *
 * <p>Constructing a subsystem registers it with the scheduler, so building this
 * inside {@code CommandOpMode.initialize()} is all it takes for periodic() to
 * start running on each of them.
 */
public class QualBotRobot {

    public final DriveSubsystem drive;
    public final ShooterSubsystem shooter;
    public final IntakeSubsystem intake;
    public final VisionSubsystem vision;
    public final LightSubsystem light;

    private final VoltageSensor batteryVoltage;

    /**
     * @param startingPose where the robot is sitting on the field, in Pedro
     *                     coordinates — build it with
     *                     {@link PedroConstants#ftcPose(double, double, double)}
     * @param pipeline     Limelight pipeline: 0 for blue, 1 for red
     */
    public QualBotRobot(HardwareMap hardwareMap, Pose startingPose, int pipeline) {
        drive = new DriveSubsystem(hardwareMap, startingPose);
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap, drive::getHeadingDegrees);
        light = new LightSubsystem(hardwareMap);

        batteryVoltage = hardwareMap.voltageSensor.get(RobotConstants.CONTROL_HUB);

        vision.start(pipeline);
    }

    /** Battery volts, for telemetry and for the voltage-dependent auto powers. */
    public double getBatteryVoltage() {
        return batteryVoltage.getVoltage();
    }

    /** True when the battery is still fresh enough for the gentler auto powers. */
    public boolean isHighVoltage() {
        return getBatteryVoltage() > RobotConstants.HIGH_VOLTAGE_THRESHOLD;
    }
}
