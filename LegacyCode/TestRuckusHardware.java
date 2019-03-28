//RENAME THE ROBOT TO SOMETHING MORE CREATIVE -PULKIT 9/27/2017
// A most amazing and thought provoking Idea - Naren Ram

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TestRuckusHardware
{
    // Here we define all of the motors we use.
    // Our robot uses omni-wheels and a four motor drive system.
    public DcMotor motorLeft;
    public DcMotor motorRight;
    public DcMotor motorFront;
    public DcMotor motorBack;
    // The motor to raise and lower the hook
    public DcMotor motorHook;

    // Here we define all of the servos we use.
    public Servo servoLeft;
    public Servo servoRight;
    // The servo to release the marker
    public Servo servoMark;

    // Defining color sensors
    ColorSensor sensorColorR = null;
    ColorSensor sensorColorL = null;
    DistanceSensor sensorRange = null;

    // Define hardware map
    HardwareMap hwMap;

    // Initialize standard hardware interfaces
    public void init(HardwareMap ahwMap) {

        // save reference to HW Map
        hwMap = ahwMap;

        // Here we name the motors.
        motorLeft  = hwMap.get(DcMotor.class, "motorLeft");
        motorRight = hwMap.get(DcMotor.class, "motorRight");
        motorBack  = hwMap.get(DcMotor.class, "motorBack");
        motorFront  = hwMap.get(DcMotor.class, "motorFront");
        motorHook  = hwMap.get(DcMotor.class, "motorHook");

        // Here we name the servos.
        servoRight = hwMap.get(Servo.class, "servoRight");
        servoLeft = hwMap.get(Servo.class, "servoLeft");
        servoMark = hwMap.get(Servo.class, "servoMark");

        // Here we set the motor directions.
        motorRight.setDirection(DcMotor.Direction.FORWARD);
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFront.setDirection(DcMotor.Direction.FORWARD);
        motorBack.setDirection(DcMotor.Direction.REVERSE);
        motorHook.setDirection(DcMotor.Direction.FORWARD);

        // Here we set the servo directions.
        servoLeft.setDirection(Servo.Direction.REVERSE);
        servoRight.setDirection(Servo.Direction.FORWARD);

        // Here we set all motors to zero power for safety.
        motorLeft.setPower(0);
        motorRight.setPower(0);
        motorFront.setPower(0);
        motorBack.setPower(0);
        motorHook.setPower(0);

        // Here we initialize the servos.
        servoLeft.setPosition(0.8);
        servoRight.setPosition(0.8);
        servoMark.setPosition(0.0);

        // Here we set all motors to run with encoders.
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorHook.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Here we initialize the sensors.
        sensorColorR = hwMap.get(ColorSensor.class, "sensorColorR");
        sensorColorL = hwMap.get(ColorSensor.class, "sensorColorL");
        sensorRange = hwMap.get(DistanceSensor.class, "sensorRange");
    }
}
