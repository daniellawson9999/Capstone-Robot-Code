//RENAME THE ROBOT TO SOMETHING MORE CREATIVE -PULKIT 9/27/2017
// A most amazing and thought provoking Idea - Naren Ram

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TempRuckusHardware_NoServo
{
    /* Public OpMode members. */
    public DcMotor motorLeft   = null;
    public DcMotor  motorRight  = null;

    //ModernRoboticsI2cRangeSensor sensorRange = null;

    /* Local OpMode members. */
    HardwareMap hwMap  = null;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        motorLeft  = hwMap.get(DcMotor.class, "motorLeft");
        motorRight = hwMap.get(DcMotor.class, "motorRight");
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorLeft.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        motorLeft.setPower(0);
        motorRight.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //Sensor stuff
        //sensorRange = hwMap.get(ModernRoboticsI2cRangeSensor.class, "range");
        //two more servos one more motor

    }
}
