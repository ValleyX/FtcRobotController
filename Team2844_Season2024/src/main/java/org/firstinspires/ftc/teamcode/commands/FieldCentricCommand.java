package org.firstinspires.ftc.team12841.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team12841.subsystems.DriveSubsystem;

/**this is where any driving is*/
public class FieldCentricCommand extends CommandBase {
    public boolean babyMode;
    private final DriveSubsystem m_driveSubsystem;
    public LinearOpMode m_opMode;
    public FieldCentricCommand(DriveSubsystem driveSubsystem, LinearOpMode opMode){
        m_driveSubsystem = driveSubsystem;
        m_opMode = opMode;
        babyMode = false;
        addRequirements(m_driveSubsystem); //prevents other command from using this subsysten while active
    }


    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        double y = -m_opMode.gamepad1.left_stick_y; // Remember, this is reversed!
        double x = m_opMode.gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = m_opMode.gamepad1.right_stick_x;

        if (m_opMode.gamepad1.x) {
            babyMode = !babyMode;
            m_opMode.sleep(200);
        }

        // Read inverse IMU heading, as the IMU heading is CW positive
        //double botHeading = -m_driveSubsytem.m_imu.getAngularOrientation().firstAngle;
        double botHeading = -m_driveSubsystem.m_imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        if (babyMode) {
            frontLeftPower *= 0.33 ;
            backLeftPower *= 0.33;
            frontRightPower *= 0.33;
            backRightPower *= 0.33;

        }
            m_driveSubsystem.m_frontLeft.setPower(frontLeftPower);
            m_driveSubsystem.m_backLeft.setPower(backLeftPower);
            m_driveSubsystem.m_frontRight.setPower(frontRightPower);
            m_driveSubsystem.m_backRight.setPower(backRightPower);


        m_opMode.telemetry.addData("rotX", rotX);
        m_opMode.telemetry.addData("rotY", rotY);
        m_opMode.telemetry.addData("front left power", frontLeftPower);
        m_opMode.telemetry.addData("front right power", frontRightPower);
        m_opMode.telemetry.addData("back left power", backLeftPower);
        m_opMode.telemetry.addData("back right power", backRightPower);
        m_opMode.telemetry.addData("denominator", denominator);
        m_opMode.telemetry.addData("baby mode", babyMode);
        m_opMode.telemetry.addData("bot heading", botHeading);
        m_opMode.telemetry.update();
    }

   /* @Override
    public boolean isFinished(){
       /* if (!m_opMode.isStopRequested() && m_opMode.opModeIsActive()) {
            return false;

        } else {
            return true;
        //}
    }*/
}