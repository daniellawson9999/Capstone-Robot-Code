//RENAME THE ROBOT TO SOMETHING MORE CREATIVE -PULKIT 9/27/2017
// A most amazing and thought provoking Idea - Naren Ram

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SensorREV2mDistanceHardware
{
    /* Public OpMode members. */
    public DcMotor motorLeft   = null;
    public DcMotor  motorRight  = null;
    DistanceSensor sensorRange = null;

    /* Local OpMode members. */
    HardwareMap hwMap  = null;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        motorLeft  = hwMap.get(DcMotor.class, "motorLeft");
        motorRight = hwMap.get(DcMotor.class, "motorRight");
        motorRight.setDirection(DcMotor.Direction.FORWARD);
        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        motorLeft.setPower(0);
        motorRight.setPower(0);

        // Set all motors to run without encoders.
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sensorRange = hwMap.get(DistanceSensor.class, "sensorRange");
    }
}
