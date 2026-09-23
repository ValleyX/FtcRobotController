package org.firstinspires.ftc.team12841.drive.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team12841.Drivers.RobotHardware;
import org.firstinspires.ftc.team12841.commands.autocommands.AutoScoreHigh;
import org.firstinspires.ftc.team12841.commands.autocommands.SubToLength;
import org.firstinspires.ftc.team12841.commands.liftcommands.LiftHighChamber;
import org.firstinspires.ftc.team12841.commands.liftcommands.LiftReset;
import org.firstinspires.ftc.team12841.commands.liftcommands.LiftToHumanPlayer;
import org.firstinspires.ftc.team12841.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team12841.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.team12841.subsystems.LiftSubsystem;


@Autonomous(group = "drive") //makes it an autonomous program
public class AutoSubexTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap); //pull in drive to use
        RobotHardware robot = new RobotHardware(this); //make an instance of robot hardware
        LiftSubsystem m_liftSub = new LiftSubsystem(robot.liftMotor, robot.hangMotor, robot.clawServo, robot.hangServo, robot.liftTouch, robot.rightBase, robot.leftBase); //making an instance of liftSub

        AutoScoreHigh autoScoreHigh = new AutoScoreHigh(m_liftSub, this);
        LiftReset liftReset = new LiftReset(m_liftSub,this);
        LiftHighChamber highChamber = new LiftHighChamber(m_liftSub,this);
        LiftToHumanPlayer humanPlayer = new LiftToHumanPlayer(m_liftSub, this);
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(robot.subExtendMotor, robot.sampleServo, robot.intakeServo);
        SubToLength subToLength = new SubToLength(intakeSubsystem,this);

        //stuff so it doesn't get funky (I think this tells the brick to stop when it says stop and go when it says go)
        waitForStart();
        if (isStopRequested()) return;

        //SET START POS
        drive.setPoseEstimate(new Pose2d(-8, -62, Math.toRadians(90)));

        //go go gadget subextend
        subToLength.initialize(10);

        sleep(1000);

    }
}
