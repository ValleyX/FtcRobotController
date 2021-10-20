package org.firstinspires.ftc.teamcode.Drivers;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class RotateToHeadingFourWheelDrive {
    private MandoRobotHardware robot_;
    private ElapsedTime runtime_;
    private boolean waiting_;
    private RotatePreciseFourWheelDrive rotatePrecise_;

    /* Constructor setup all class variables here */
    public RotateToHeadingFourWheelDrive(MandoRobotHardware robot, RotatePreciseFourWheelDrive rotatePrecise) {
        robot_ = robot;
        runtime_ = new ElapsedTime();
        waiting_ = false;
        rotatePrecise_ = rotatePrecise;

        /* ---new remapping code --*/
        //swapping y & z axis due to vertical mounting of rev expansion board
        byte AXIS_MAP_CONFIG_BYTE = 0x6; //This is what to write to the AXIS_MAP_CONFIG register to swap x and z axes
        //byte AXIS_MAP_CONFIG_BYTE = 0x18; //This is what to write to the AXIS_MAP_CONFIG register to swap y and z axes
        byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis

        //Need to be in CONFIG mode to write to registers
        robot_.imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal);
        robot_.OpMode_.sleep(100); //Changing modes requires a delay before doing anything else

        //Write to the AXIS_MAP_CONFIG register
        robot_.imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, AXIS_MAP_CONFIG_BYTE);

        //Write to the AXIS_MAP_SIGN register
        robot_.imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN, AXIS_MAP_SIGN_BYTE);

        //Need to change back into the IMU mode to use the gyro
        robot_.imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal & 0x0F);
        robot_.OpMode_.sleep(100); //Changing modes again requires a delay

        /* ---new remapping code ---*/
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); // inertial motion unit
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        robot_.imu.initialize(parameters);

        // make sure the imu gyro is calibrated before continuing.
        while (!robot_.OpMode_.isStopRequested() && !robot_.imu.isGyroCalibrated()) {
            robot_.OpMode_.sleep(50);
            robot.OpMode_.idle();
        }

        if (!robot_.imu.isGyroCalibrated()) {
            System.out.println("ValleyX: Gyro not calibrated");
        }

        double gyroActual = robot_.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public void DoIt(double heading) {
        DoItSpecify(heading, 2, 0.4, 0.3, 5);
    }

    public void DoItSpecify(double heading, double gyroRange, double minSpeed, double addSpeed, int timesCorrect) {
        //0 turns the wrong way force to 0.1
        if (heading == 0.0) {
            heading = 0.1;
        }

        double gyroActual = robot_.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        gyroActual = 360 - gyroActual;
        gyroActual += 360.0;
        gyroActual %= 360;
        double turnAngle = heading - gyroActual;

        if (turnAngle > 180.0) turnAngle -= 360.0; //makes delta between -180 and 180
        if (turnAngle < -180.0) turnAngle += 360.0; //makes delta between -180 and 180

        rotatePrecise_.RotatePrecise(turnAngle, gyroRange, minSpeed, addSpeed, timesCorrect);
    }
}