package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="BringUp", group="Zippo")
public class ZoBringUp extends ZoDriving {
    public void runOpMode()
    {
        super.runTf = false;
        super.runOpMode();
        MoveHookUp(true);
    }
}
