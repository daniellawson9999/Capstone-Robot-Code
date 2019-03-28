package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.DrawViewSource;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ZoDriving;

@Autonomous(name="CraterRegionals", group="Zippo")
public class ZoCraterRegionals extends ZoDriving {
    public void runOpMode()
    {
        /*
        STEPS:
        READ WHILE HANGING
        LAND
        MOVE OUT AND OF THE HOOK AND MOVE BACK CENTER
        KNOCK DOWN THE RIGHT MINERAL AND PARK
        */

        //inherent initialization from the ZoDriving class, this saves code redundancy
        super.runOpMode();
        //Start the custom tensorflow object which has already been creatd in the super class
        tf.start();
        //Pause for a second and read the mineral orientation while hanging
        sleep(1000);
        TensorFlow.MineralLocation goldMineralLocation = tf.nMineralLocations(TensorFlow.RobotOrientation.Left,5);
        //TensorFlow.MineralLocation goldMineralLocation = tf.getMineralLocation(TensorFlow.RobotOrientation.Left);
        sleep(250);
        tf.shutdown();
        //Lower the robot (A boolean decides the direction, passing false lowers the robot)
        MoveHookUp(false);
        sleep(250);
        telemetry.addData("Mineral Location: ", goldMineralLocation);
        telemetry.update();

        //Move out of the hook to the left, drive forward, and move back towards the center
        latEncoderDrive(.6,  -5,  -5, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        encoderDrive(DRIVE_SPEED, -6, -6, 7);

        //The three if statements for handling logic depending on the mineral location
        if(goldMineralLocation == TensorFlow.MineralLocation.Left)
        {
            //latEncoderDrive(0.6, 1, 1, 5);
            encoderDrive(DRIVE_SPEED, -9.5, -9.5, 5);
            gyroDrive(113, xyz, -0.5, 5);
            latEncoderDrive(DRIVE_SPEED, 25, 25, 5);
            encoderDrive(DRIVE_SPEED, -10, -10, 3);
            while(robot.sensorRangeR.getDistance(DistanceUnit.INCH) > 7){
                latEncoderDrive(1,4,4,3);
            }
            encoderDrive(DRIVE_SPEED, -58, -58, 5);
            releaseMarker();
            encoderDrive(DRIVE_SPEED, 75, 75, 5);
        }
        else if(goldMineralLocation == TensorFlow.MineralLocation.Center)
        {
            latEncoderDrive(0.6, 5.5, 5.5, 5);
            //drive straight and park if center
            encoderDrive(DRIVE_SPEED, -25, -25, 7); //forward and hit mineral
            encoderDrive(DRIVE_SPEED, 10, 10, 7);
            latEncoderDrive(DRIVE_SPEED, -30, -30, 5);
            gyroDrive(113, xyz, -0.5, 5); //turn towards mineral
            while(robot.sensorRangeR.getDistance(DistanceUnit.INCH) > 7){
                latEncoderDrive(1,4,4,3);
            }
            encoderDrive(DRIVE_SPEED, -58, -58, 5);
            releaseMarker();
            encoderDrive(DRIVE_SPEED, 75, 75, 5);
        }
        else
        {
            latEncoderDrive(0.6, 15, 15, 5);
            //drive straight and park if center
            encoderDrive(DRIVE_SPEED, -25, -25, 7); //forward and hit mineral
            encoderDrive(DRIVE_SPEED, 10, 10, 7);
            latEncoderDrive(DRIVE_SPEED, -40, -40, 5);
            gyroDrive(113, xyz, -0.5, 5); //turn towards mineral
            while(robot.sensorRangeR.getDistance(DistanceUnit.INCH) > 7){
                latEncoderDrive(1,4,4,3);
            }
            encoderDrive(DRIVE_SPEED, -58, -58, 5);
            releaseMarker();
            encoderDrive(DRIVE_SPEED, 75, 75, 5);
        }
    }
}