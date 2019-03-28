package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.TensorFlow.RobotOrientation;

@Autonomous(name="CustomTensorFlowTest", group="Zippo")
public class CustomTensorFlowObjectDection extends ZoDriving {
    public void runOpMode()
    {
        super.runOpMode();
        tf.start();
        while(opModeIsActive()){
            telemetry.addData("value",tf.getMineralLocation(RobotOrientation.Left));
            telemetry.update();
        }
        tf.shutdown();

    }
}
