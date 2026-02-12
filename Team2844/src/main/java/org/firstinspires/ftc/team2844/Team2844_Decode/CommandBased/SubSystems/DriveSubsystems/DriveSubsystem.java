package org.firstinspires.ftc.team2844.Team2844_Decode.CommandBased.SubSystems.DriveSubsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;

//import org.firstinspires.ftc.team2844.Team2844_Decode.RoadrunnerStuff.RoadrunnerQuickstart.MecanumDrive;

public class DriveSubsystem extends SubsystemBase {

    private MecanumDrive drive;
    private Motor frontLeft, frontRight, backLeft, backRight;

    public DriveSubsystem(Motor frontLeft, Motor frontRight,Motor backLeft,Motor backRight){

        this.frontLeft=frontLeft;
        this.frontRight=frontRight;
        this.backLeft=backLeft;
        this.backRight=backRight;

        drive = new MecanumDrive(frontLeft,frontRight,backLeft,backRight);

    }

    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed, double heading) {
        drive.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, heading);
    }

    /* this is a option for the subsystem
    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    }*/
}
