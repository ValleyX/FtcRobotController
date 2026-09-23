package org.firstinspires.ftc.team12841.testcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


        import org.firstinspires.ftc.team12841.Drivers.RobotHardware;
        import org.firstinspires.ftc.team12841.Drivers.OdometryDrive;

//robot must pe put in the same place to relatively same position per game to be consistant
//Position distance from corner near board is about 48 inches
@Disabled
@Autonomous(name="AprilAutoTest")
public class AprilAutoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        //Using test hardware for now because test hardware has properties needed for practice
        RobotHardware robot = new RobotHardware(this,true); //checkBlue is true to only find
        OdometryDrive odometryDrive = new OdometryDrive(robot);
        RobotHardware.CenterStagePipeline.DetectionPosition position = RobotHardware.CenterStagePipeline.DetectionPosition.Left; // position robot detects
        while(opModeInInit()){
            //to tell user what values the camera sees
            telemetry.addData("r value", robot.pipeline.avgR);
            telemetry.addData("b value", robot.pipeline.avgB);
            telemetry.addData("r2 value", robot.pipeline.avg2R);
            telemetry.addData("b2 value", robot.pipeline.avg2B);
            telemetry.addData("r3 value", robot.pipeline.avg3R);
            telemetry.addData("b3 value", robot.pipeline.avg3B);

            telemetry.addData("teamProp position", robot.pipeline.position);

            telemetry.update();

            position = robot.pipeline.position;


        }
        position = robot.pipeline.position;



        //robot sees the marker is on the left position
        if(position == RobotHardware.CenterStagePipeline.DetectionPosition.Left){


            //moves forward     to spikemarks
            odometryDrive.goToPositionForward(0, 24, 0.5, 0, 0.5);

            //take a chill pill
            sleep(100);

            //turns 90deg to left       cntr clockwise
            odometryDrive.changeRobotOrientation(0.5, -90, 3.25);

            //take a chill pill
            sleep(100);

            //moves forward     to board
            odometryDrive.goToPositionForward(0, 24, 0.5, -90, 0.5);


            //APRIL TAG CODEEEEE




        }




        //middle
        if(position == RobotHardware.CenterStagePipeline.DetectionPosition.Middle){


            //moves forward     to spikemarks
            odometryDrive.goToPositionForward(0, 24, 0.5, 0, 0.5);

            //take a chill pill
            sleep(100);

            //turns 90deg to left       cntr clockwise
            odometryDrive.changeRobotOrientation(0.5, -90, 3.25);

            //take a chill pill
            sleep(100);

            //moves forward     to board
            odometryDrive.goToPositionForward(0, 24, 0.5, -90, 0.5);


        }




        //right
        if(position == RobotHardware.CenterStagePipeline.DetectionPosition.Right){


            //moves forward     to spikemarks
            odometryDrive.goToPositionForward(0, 24, 0.5, 0, 0.5);

            //take a chill pill
            sleep(100);

            //turns 90deg to left       cntr clockwise
            odometryDrive.changeRobotOrientation(0.5, -90, 3.25);

            //take a chill pill
            sleep(100);

            //moves forward     to board
            odometryDrive.goToPositionForward(0, 24, 0.5, -90, 0.5);


        }




    }


}