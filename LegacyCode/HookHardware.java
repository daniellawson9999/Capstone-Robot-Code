//RENAME THE ROBOT TO SOMETHING MORE CREATIVE -PULKIT 9/27/2017
// A most amazing and thought provoking Idea - Naren Ram

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HookHardware
{
    /* Public OpMode members. */
    public DcMotor motorLeft;


    /* Local OpMode members. */
    HardwareMap hwMap;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        motorLeft  = hwMap.get(DcMotor.class, "motorLeft");
        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorLeft.setPower(0);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
