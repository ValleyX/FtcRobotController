package org.firstinspires.ftc.team12841;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.team12841.pedroPathing.Constants;
import com.pedropathing.follower.Follower;

import java.util.List;

public class RobotHardware {

    public final OpMode opMode;
    private final ElapsedTime runtime = new ElapsedTime();

    // Drive Motors
    public DcMotorEx rfMotor, rbMotor, lfMotor, lbMotor;

    // Shooter
    public DcMotorEx shooterMotor;

    // Odometry Encoders
    public DcMotorEx leftOdo, rightOdo, strafeOdo;

    // Servos
    public Servo turntableServo, shooterServo;

    // Sensors
    public IMU imu;
    public Limelight3A limelight;

    // Pedro
    private Follower follower;

    //motor speed adding
    public double lFDriveSpeed = 0.0;
    public double rFDriveSpeed = 0.0;
    public double lBDriveSpeed = 0.0;
    public double rBDriveSpeed = 0.0;

    public double lFAlignSpeed = 0.0;
    public double rFAlignSpeed = 0.0;
    public double lBAlignSpeed = 0.0;
    public double rBAlignSpeed = 0.0;



    public final double TURN_THRESH = 2.3;
    public final double SLOW_THRESH = 15;
    public final double PGAIN = 0.023;


    public RobotHardware(OpMode opMode) {
        this.opMode = opMode;

        // MOTORS + ODOMETRY
        try {
            rfMotor = opMode.hardwareMap.get(DcMotorEx.class, "rfMotor");
            rbMotor = opMode.hardwareMap.get(DcMotorEx.class, "rbMotor");
            lfMotor = opMode.hardwareMap.get(DcMotorEx.class, "lfMotor");
            lbMotor = opMode.hardwareMap.get(DcMotorEx.class, "lbMotor");

            shooterMotor = opMode.hardwareMap.get(DcMotorEx.class, "shooterMotor");

            leftOdo   = opMode.hardwareMap.get(DcMotorEx.class, "leftOdo");
            rightOdo  = opMode.hardwareMap.get(DcMotorEx.class, "rightOdo");
            strafeOdo = opMode.hardwareMap.get(DcMotorEx.class, "strafeOdo");

        } catch (Exception e) {
            opMode.telemetry.addLine("ERROR: Motor/Odo mapping failed!");
        }

        // SERVOS
        try {
            shooterServo = opMode.hardwareMap.get(Servo.class, "shooterServo");
            turntableServo = opMode.hardwareMap.get(Servo.class, "turntableServo");
        } catch (Exception e) {
            opMode.telemetry.addLine("ERROR: Servo mapping failed!");
        }

        // IMU
        try {
            imu = opMode.hardwareMap.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )
            ));
        } catch (Exception e) {
            opMode.telemetry.addLine("ERROR: IMU failed!");
        }

        // LIMELIGHT STUFF
        try {
            limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0);   // default pipeline
            limelight.start();             // begin streaming
            opMode.telemetry.addLine("Limelight3A ONLINE");
        } catch (Exception e) {
            limelight = null;
            opMode.telemetry.addLine("Limelight3A NOT FOUND");
        }

        // MOTOR CONFIG
        try {
            lfMotor.setDirection(DcMotor.Direction.REVERSE);
            lbMotor.setDirection(DcMotor.Direction.REVERSE);

            rfMotor.setDirection(DcMotor.Direction.FORWARD);
            rbMotor.setDirection(DcMotor.Direction.FORWARD);

            for (DcMotorEx m : new DcMotorEx[]{lfMotor, lbMotor, rfMotor, rbMotor}) {
                m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            for (DcMotorEx odo : new DcMotorEx[]{leftOdo, rightOdo, strafeOdo}) {
                odo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                odo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                odo.setPower(0);
            }

        } catch (Exception e) {
            opMode.telemetry.addLine("ERROR: motor/encoder config failed!");
        }

        // PEDRO FOLLOWER
        try {
            follower = Constants.createFollower(opMode.hardwareMap);
            opMode.telemetry.addLine("Pedro Pathing OK");
        } catch (Exception e) {
            follower = null;
            opMode.telemetry.addLine("Pedro FAILED");
        }

        opMode.telemetry.update();
    }

    private LinearOpMode opMode_;

    LLResult llResult;
    boolean pipelineCalled = false;

    public void innit(int pipeline){
        limelight.pipelineSwitch(pipeline);
        limelight.start();
        pipelineCalled = true;
    }

    public int getPipeline(){
        return limelight.getStatus().getPipelineIndex();
    }
    public LLResult getLatestResult(){
        return limelight.getLatestResult();
    }

    public void updateResult(){
        llResult = limelight.getLatestResult();
    }

    public void updateIMU(double IMUHeading){
        limelight.updateRobotOrientation(IMUHeading);
    }

    //Returns the y of the bot on the field through metatags
    public double getBotY(){
        updateResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose();
            return botPose.getPosition().y*254;
        }
        return -999;
    }


    //Returns the X of the bot on the field through metatags
    public double getBotX(){
        updateResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose();
            return botPose.getPosition().x*254;
        }
        return -999;
    }

    public double getBotZ(){
        updateResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose();
            return botPose.getPosition().z*254;
        }
        return -999;
    }

    public double getBotDis(){
        updateResult();
        if (llResult != null && llResult.isValid()) {
            return 68.86747*Math.pow(llResult.getTa(), -0.5169279); //using the equation we got from the graph we insert the ta as the x and return distance as y
        }
        return -999;
    }

    //Returns the rotation of the bot on the field through metatags
    public double getBotRot(){
        updateResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose();
            return botPose.getPosition().z;
        }
        return -999;
    }

    public double getBotCamZ(){
        updateResult();
        if (llResult != null && llResult.isValid()) {
            List< LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
            return ((fiducials.get(0).getCameraPoseTargetSpace().getPosition().z *100)/2.54)*-1.635;
        }
        return -999;
    }

    public double getBotCamY(){
        updateResult();
        if (llResult != null && llResult.isValid()) {
            List< LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
            return ((fiducials.get(0).getCameraPoseTargetSpace().getPosition().y *100)/2.54)*-1.635;
        }
        return -999;
    }

    public double getBotCamX(){
        updateResult();
        if (llResult != null && llResult.isValid()) {
            List< LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
            return ((fiducials.get(0).getCameraPoseTargetSpace().getPosition().x *100)/2.54)*-1.635;
        }
        return -999;
    }



    public double getTy(){
        updateResult();
        if (llResult != null && llResult.isValid()) {
            return llResult.getTy();
        }
        return -999;
    }

    public double getTx(){
        updateResult();
        if (llResult != null && llResult.isValid()) {
            return llResult.getTx();
        }
        return -999;
    }

    public double getTarea(){
        updateResult();
        if (llResult != null && llResult.isValid()) {
            return llResult.getTa();
        }
        return -999;
    }

    public Follower getFollower() {
        return follower;
    }

    public void resetIMU() {
        if (imu != null) imu.resetYaw();
    }

    // *******************************CARTERS STUFF*************************************88
    public void powerMotors(double leftFront, double leftBack, double rightBack, double rightFront){

        lfMotor.setPower(leftFront);
        lbMotor.setPower(leftBack);
        rfMotor.setPower(rightFront);
        rbMotor.setPower(rightBack);
    }

    public void addDrivePower(double leftFront, double leftBack, double rightBack, double rightFront){
        lFDriveSpeed = leftFront;
        lBDriveSpeed = leftBack;
        rFDriveSpeed = rightFront;
        rBDriveSpeed = rightBack;

        if(Math.abs(lFDriveSpeed) >= 1){
            lFDriveSpeed /= Math.abs(lFDriveSpeed);
        }
        if(Math.abs(lBDriveSpeed) >= 1){
            lBDriveSpeed /= Math.abs(lBDriveSpeed);
        }
        if(Math.abs(rFDriveSpeed) >= 1){
            rFDriveSpeed /= Math.abs(rFDriveSpeed);
        }
        if(Math.abs(rBDriveSpeed) >= 1){
            rBDriveSpeed /= Math.abs(rBDriveSpeed);
        }
        calculateDrive();
    }

    public void addAlignPower(double leftFront, double leftBack, double rightBack, double rightFront){
        lFAlignSpeed = leftFront;
        lBAlignSpeed = leftBack;
        rFAlignSpeed = rightFront;
        rBAlignSpeed = rightBack;

        if(Math.abs(lFAlignSpeed) >= 1){
            lFAlignSpeed /= Math.abs(lFAlignSpeed);
        }
        if(Math.abs(lBAlignSpeed) >= 1){
            lBAlignSpeed /= Math.abs(lBAlignSpeed);
        }
        if(Math.abs(rFAlignSpeed) >= 1){
            rFAlignSpeed /= Math.abs(rFAlignSpeed);
        }
        if(Math.abs(rBAlignSpeed) >= 1){
            rBAlignSpeed /= Math.abs(rBAlignSpeed);
        }

        calculateDrive();
    }

    public void turnToFree(double degrees, double speed){
        double oppDeg;
        double mult;
        double heading = robotHeadingAngles();

        mult = Math.abs(Math.abs(degrees) - Math.abs(heading)) * PGAIN;
        if(!( degrees - TURN_THRESH < heading && heading < degrees + TURN_THRESH)) {
            if (degrees > 0) {
                oppDeg = degrees - 180;
                if (oppDeg < heading && heading < degrees) {
                    addAlignPower(Math.min(-speed * mult, 1), Math.min(-speed * mult, 1), Math.min(speed * mult, 1), Math.min(speed * mult, 1));
                } else  if(degrees < heading && heading < oppDeg){
                    addAlignPower(Math.min(speed * mult, 1), Math.min(speed * mult, 1), Math.min(-speed * mult, 1), Math.min(-speed * mult, 1));
                } else {
                    addAlignPower(0,0,0,0);
                }
            } else {
                oppDeg = degrees + 180;
                if (degrees < heading && heading < oppDeg) {
                    addAlignPower(Math.min(speed * mult, 1), Math.min(speed * mult, 1), Math.min(-speed * mult, 1), Math.min(-speed * mult, 1));
                } else if (oppDeg < heading && heading < degrees) {
                    addAlignPower(Math.min(-speed * mult, 1), Math.min(-speed * mult, 1), Math.min(speed * mult, 1), Math.min(speed * mult, 1));
                } else {
                    addAlignPower(0,0,0,0);
                }
            }
        } else {
            addAlignPower(0,0,0,0);
        }
    }

    public void calculateDrive(){
        double leftFront = lFAlignSpeed + lFDriveSpeed;
        double leftBack = lBAlignSpeed + lBDriveSpeed;
        double rightFront = rFAlignSpeed + rFDriveSpeed;
        double rightBack = rBAlignSpeed + rBDriveSpeed;

        if(Math.abs(leftFront) >= 1){
            leftFront /= Math.abs(leftFront);
        }
        if(Math.abs(leftBack) >= 1){
            leftBack /= Math.abs(leftBack);
        }
        if(Math.abs(rightFront) >= 1){
            rightFront /= Math.abs(rightFront);
        }
        if(Math.abs(rightBack) >= 1){
            rightBack /= Math.abs(rightBack);
        }

        powerMotors(leftFront, leftBack, rightBack, rightFront);
    }

    public double robotHeadingRadians(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public double robotHeadingAngles(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void resetImu(){
        imu.resetYaw();
    }


    public void alignFree(double llx) {
        //Use the Limelight llx to fine tune without stoping the driver
        if (llx != -999){
            turnToFree(robotHeadingAngles() - llx, .6);
        }
    }

    public void align(double llx){
        //Use the Limelight llx to fine tune
        if(llx != -999) {
            turnTo(robotHeadingAngles() - llx, .6);
        }
    }

    public void turnToEstimate(boolean red){
        //use the IMU to go to the estimated degrees
        double degrees = 0;
        if(red){
            //if we are red side and IMU is set to the back, turn sorta towards the goal
            degrees = -45;
        } else {
            degrees = 45;
        }

        turnToFree(degrees, 0.5);

    }

    public void turnTo(double degrees, double speed){
        double oppDeg;
        double mult;
        double heading = robotHeadingAngles();

        mult = Math.abs(Math.abs(degrees) - Math.abs(heading)) * PGAIN;
        while(!( degrees - TURN_THRESH < heading && heading < degrees + TURN_THRESH) && opMode_.opModeIsActive()) {
            heading = robotHeadingAngles();
            if (degrees > 0) {
                oppDeg = degrees - 180;
                if (oppDeg < heading && heading < degrees) {
                    addAlignPower(Math.min(-speed * mult, 1), Math.min(-speed * mult, 1), Math.min(speed * mult, 1), Math.min(speed * mult, 1));
                } else  if(degrees < heading && heading < oppDeg){
                    addAlignPower(Math.min(speed * mult, 1), Math.min(speed * mult, 1), Math.min(-speed * mult, 1), Math.min(-speed * mult, 1));
                } else {
                    addAlignPower(0,0,0,0);
                }
            } else {
                oppDeg = degrees + 180;
                if (degrees < heading && heading < oppDeg) {
                    addAlignPower(Math.min(speed * mult, 1), Math.min(speed * mult, 1), Math.min(-speed * mult, 1), Math.min(-speed * mult, 1));
                } else if (oppDeg < heading && heading < degrees) {
                    addAlignPower(Math.min(-speed * mult, 1), Math.min(-speed * mult, 1), Math.min(speed * mult, 1), Math.min(speed * mult, 1));
                } else {
                    addAlignPower(0,0,0,0);
                }
            }
        }
        addAlignPower(0,0,0,0);
    }
}
