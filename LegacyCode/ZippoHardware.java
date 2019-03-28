//RENAME THE ROBOT TO SOMETHING MORE CREATIVE -PULKIT 9/27/2017
// A most amazing and thought provoking Idea - Naren Ram

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class ZippoHardware
{
    /* Public OpMode members. */
    public DcMotor motorLeft   = null;
    public DcMotor  motorRight  = null;
    public DcMotor motorLateral = null;
    public DcMotor motorPulley = null;
    public DcMotor motorRelic = null;
    public DcMotor motorRelicVertical = null;
    //public DcMotor name = null;
    public Servo servoArm = null;
    public Servo servoLeft = null;
    public Servo servoRight = null;
    public Servo servoRelic = null;
    //public Servo servoClaw = null;
    ColorSensor sensorColor = null;
    //ModernRoboticsI2cRangeSensor sensorRange = null;
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
        motorLateral = hwMap.get(DcMotor.class, "motorLateral");
        motorPulley = hwMap.get(DcMotor.class, "motorPulley");
        motorRelic = hwMap.get(DcMotor.class, "motorRelic");
        motorRelicVertical = hwMap.get(DcMotor.class, "motorRelicVertical");
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorLateral.setDirection(DcMotor.Direction.REVERSE);
        motorPulley.setDirection(DcMotor.Direction.FORWARD);
        motorRelic.setDirection(DcMotor.Direction.FORWARD);
        motorRelicVertical.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        motorLeft.setPower(0);
        motorRight.setPower(0);
        motorLateral.setPower(0);
        motorPulley.setPower(0);
        motorRelic.setPower(0);
        motorRelicVertical.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLateral.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorPulley.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRelic.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRelicVertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        servoArm = hwMap.get(Servo.class, "servoArm");
        servoLeft = hwMap.get(Servo.class, "servoLeft");
        servoRight = hwMap.get(Servo.class, "servoRight");
        servoRelic = hwMap.get(Servo.class, "servoRelic");

        servoRelic.setPosition(0.5);
        servoLeft.setPosition(0.2); //l1 r 0
        servoRight.setPosition(0.2);

        //Sensor stuff
        //sensorRange = hwMap.get(ModernRoboticsI2cRangeSensor.class, "range");
        //two more servos one more motor
        sensorColor = hwMap.get(ColorSensor.class, "sensorColor");
        sensorRange = hwMap.get(DistanceSensor.class, "sensorRange");
    }
}
