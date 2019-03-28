package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="SensorTest", group="Zippo")
public class DistanceSensorTest extends ZoDriving {
    public void runOpMode()
    {
        super.runOpMode();
        while(opModeIsActive()){
            //double left = sensorRangeL.getDistance(DistanceUnit.INCH);
            double left = robot.sensorRangeL.getDistance(DistanceUnit.INCH);
            //double right = sensorRangeL.getDistance(DistanceUnit.INCH);
            double right = robot.sensorRangeR.getDistance(DistanceUnit.INCH);
            telemetry.addData("Distance Readings: ", "Left: " + left + " Right: " + right);
            telemetry.update();
        }
    }
}
