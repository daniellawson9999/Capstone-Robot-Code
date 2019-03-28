//RENAME THE ROBOT TO SOMETHING MORE CREATIVE -PULKIT 9/27/2017
// A most amazing and thought provoking Idea - Naren Ram

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;

public class ZoHardware
{
    // Here we define all of the motors we use.
    // Our robot uses omni-wheels and a four motor drive system.
    public DcMotor motorLeft;
    public DcMotor motorRight;
    public DcMotor motorFront;
    public DcMotor motorBack;
    // The motor to raise and lower the hook
    public DcMotor motorHook;
    public DcMotor motorArmExt;
    public DcMotor motorArmTilt;
    public DcMotor motorLiftBucket;

    // The servo to release the marker
    public Servo servoMark;
    public CRServo servoIntake;
    public CRServo servoBox;

    // Defining color sensors
    Rev2mDistanceSensor sensorRangeR = null;
    Rev2mDistanceSensor sensorRangeL = null;

    // Define hardware map
    HardwareMap hwMap;

    // Initialize standard hardware interfaces
    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        // Here we name the motors.
        motorLeft  = hwMap.get(DcMotor.class, "motorLeft");
        motorRight = hwMap.get(DcMotor.class, "motorRight");
        motorBack  = hwMap.get(DcMotor.class, "motorBack");
        motorFront  = hwMap.get(DcMotor.class, "motorFront");
        motorHook  = hwMap.get(DcMotor.class, "motorHook");
        motorArmExt  = hwMap.get(DcMotor.class, "motorArmExt");
        motorArmTilt  = hwMap.get(DcMotor.class, "motorArmTilt");
        motorLiftBucket = hwMap.get(DcMotor.class, "motorLiftBucket");
        // Here we name the servos.
        servoMark = hwMap.get(Servo.class, "servoMark");
        servoIntake = hwMap.get(CRServo.class, "servoIntake");
        servoBox = hwMap.get(CRServo.class, "servoBox");

        // Here we set the motor directions.
        motorRight.setDirection(DcMotor.Direction.FORWARD);
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFront.setDirection(DcMotor.Direction.FORWARD);
        motorBack.setDirection(DcMotor.Direction.REVERSE);
        motorHook.setDirection(DcMotor.Direction.FORWARD);
        motorArmExt.setDirection(DcMotor.Direction.FORWARD);
        motorArmTilt.setDirection(DcMotor.Direction.FORWARD);
        motorLiftBucket.setDirection(DcMotor.Direction.FORWARD);

        // Here we set the servo directions.

        // Here we set all motors to zero power for safety.
        motorLeft.setPower(0);
        motorRight.setPower(0);
        motorFront.setPower(0);
        motorBack.setPower(0);
        motorHook.setPower(0);
        motorArmExt.setPower(0);
        motorArmTilt.setPower(0);
        motorLiftBucket.setPower(0);
        // Here we initialize the servos.
        servoMark.setPosition(0.9);
        servoIntake.setPower(0.0);
        servoBox.setPower(0);

        // Here we set all motors to run with encoders.
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorHook.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArmExt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArmTilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLiftBucket.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Here we initialize the sensors
        sensorRangeR = hwMap.get(Rev2mDistanceSensor.class, "sensorRangeR");
        sensorRangeL = hwMap.get(Rev2mDistanceSensor.class, "sensorRangeL");


    }
}
