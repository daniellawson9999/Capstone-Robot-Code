package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.teamcode.TensorFlow.RobotOrientation;
import org.firstinspires.ftc.teamcode.TensorFlow.MineralLocation;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ZoDriving;

@Autonomous(name="Depot", group="Zippo")
public class ZoDepot extends ZoDriving {
    public void runOpMode()
    {
        super.runOpMode();
        //Start the custom tensorflow object which has already been creatd in the super class
        tf.start();
        //Pause for a second and read the mineral orientation while hanging
        sleep(1000);
        TensorFlow.MineralLocation goldMineralLocation = tf.nMineralLocations(TensorFlow.RobotOrientation.Left,5);
        sleep(250);
        tf.shutdown();
        //Lower the robot (A boolean decides the direction, passing false lowers the robot)
        MoveHookUp(false);
        sleep(250);
        telemetry.addData("Mineral Location: ", goldMineralLocation);
        telemetry.update();

        //Move out of the hook to the left, drive forward, and move back towards the center
        latEncoderDrive(.6,  -6,  -6, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        encoderDrive(DRIVE_SPEED, -5, -5, 5);


        //uncomment line after problem is resolved
        //MineralLocation goldMineralLocation = MineralLocation.Left;

        if(goldMineralLocation == MineralLocation.Left)
        {
            /*
            latEncoderDrive(DRIVE_SPEED, 2.5, 2.5, 5);
            gyroDrive(26, xyz, -0.5, 5);
            encoderDrive(DRIVE_SPEED, -40, -40, 5);
            gyroDrive(45, xyz, -0.5, 7);
            //0
            // encoderDrive(DRIVE_SPEED, -4, -4, 5);
            latEncoderDrive(.7,30,30,5);
            releaseMarker();
            //encoderDrive(DRIVE_SPEED, -20, -20, 5);
            */
            encoderDrive(DRIVE_SPEED,-1,-1,3);
            latEncoderDrive(0.6, -5, -5, 5);
            //gyroDrive(-22, xyz, -0.5, 5);
            encoderDrive(DRIVE_SPEED,-30,-30,3);
            while(robot.sensorRangeL.getDistance(DistanceUnit.INCH) > 6){
                encoderDrive(1,-4,-4,3);
                telemetry.addData("Distance: ", robot.sensorRangeL.getDistance(DistanceUnit.INCH));

            }
            //enc oderDrive(DRIVE_SPEED, -38, -38, 5);
            gyroDrive(45, xyz, -0.7, 5);
            latEncoderDrive(DRIVE_SPEED, 25, 25, 5);
            releaseMarker();
        }

        else if(goldMineralLocation == MineralLocation.Center)
        {
            latEncoderDrive(DRIVE_SPEED, 7, 7, 5);
            encoderDrive(DRIVE_SPEED, -50, -55, 5);
            releaseMarker();
            gyroDrive(45, xyz, -0.3, 5);

        }

        else
        {
            encoderDrive(DRIVE_SPEED,-1,-1,3);
            latEncoderDrive(0.6, 17, 17, 5);
            //gyroDrive(-22, xyz, -0.5, 5);
            encoderDrive(DRIVE_SPEED,-30,-30,3);
            while(robot.sensorRangeL.getDistance(DistanceUnit.INCH) > 16){
                encoderDrive(1,-5,-5,3);
                telemetry.addData("Distance: ", robot.sensorRangeL.getDistance(DistanceUnit.INCH));

            }
            //enc oderDrive(DRIVE_SPEED, -38, -38, 5);
            gyroDrive(45, xyz, -0.7, 5);
            encoderDrive(DRIVE_SPEED, -20, -20, 5);
            releaseMarker();
        }

        //align with wall
        //double tdistance = 5;
        //double cdistance = robot.sensorRangeL.getDistance(DistanceUnit.INCH);
        //latEncoderDrive(DRIVE_SPEED,-(cdistance-tdistance),-(cdistance-tdistance),3);
        //encoderDrive(1,-10,-10,3);
        while(robot.sensorRangeL.getDistance(DistanceUnit.INCH) > 6){
            encoderDrive(1,-4,-4,3);
        }

        //drive to crater
        latEncoderDrive(1,-80,-80,10);

        //extend arm into crater [ADD LATER WHEN WE HAVE ARM]

        //shutdown tensorflow (necessary)
    }
}
