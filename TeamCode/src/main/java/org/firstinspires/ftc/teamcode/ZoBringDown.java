package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BringDown", group="Zippo")
public class ZoBringDown extends ZoDriving {
    public void runOpMode()
    {
        super.runOpMode();
        MoveHookUp(false);
    }
}
