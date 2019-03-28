package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.TensorFlow.RobotOrientation;

@Autonomous(name="AngleTest", group="Zippo")
public class AngleTest extends ZoDriving {
    public void runOpMode()
    {
        super.runOpMode();
        tf.start();
        while(opModeIsActive()){
            telemetry.addData("Angle Readings: ", "x: " + readAngle("x") + " y: " + readAngle("y") + " z: " + readAngle("z"));
            telemetry.update();
        }
        tf.shutdown();

    }
}
