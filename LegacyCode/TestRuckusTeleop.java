package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="RuckusTeleOp", group="Zippo")
//@Disabled

//THE KEY BELOW IS FOR USING IMAGE RECOGNITION -PULKIT
//AT5CUvf/////AAAAGaBn6TlejU79iRr5dpGz0Msa4+WbMquS0c0rHQGMURBOGIxPznOmaavjYRYfWHE/qRnpaHDvKIVV1drOmZguwKjiTVfUzVpkRgxdFzcVDsNBldxzhrcSl+bRKGlNv3zKHDfaOJioTa7uzIN/uKUzdJPX+o5PQQxRPYXBuIvAkASbZ9/MVjq5u3Jltyw3Gz9DCPVgxqcMKILOwv9FpMDMRTcgeRwk7f+pPd8f5FmB8ehr3xiDbPxydmYAkpuqQ6Mx2qiggbSlzl4uTm2JeqOP3hbej+ozcevtHKh9C4S3eKodfDUpKekBfdOuR2aer0FwrWxfAqmdOewy5Tei71lLAOgEzx+vo6OPKpSzbTh1gFzI

public class TestRuckusTeleop extends OpMode{

    TestRuckusHardware robot  = new TestRuckusHardware();

    // Create variables for motor power
    private double lPower = 0; //left wheel
    private double rPower = 0; //right wheel
    private double fPower = 0; //front power
    private double bPower = 0; //back power
    private double hPower = 0; //hook power

    private double servoLeftPos = 0.5;
    private double servoRightPos = 0.5;

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        //robot.sensorColor.enableLed(false);
    }

    @Override
    public void loop() {

//      MAIN DRIVING CONTROLS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        lPower = gamepad1.right_stick_y;
        rPower = gamepad1.right_stick_y;
        fPower = gamepad1.right_stick_x;
        bPower = gamepad1.right_stick_x;

        if(gamepad1.dpad_up)
        {
            hPower = -1;
        }
        else if(gamepad1.dpad_down)
        {
            hPower = 1;
        }
        else
        {
            hPower = 0;
        }

        //rotate
        if(gamepad1.left_stick_x > 0)
        {
            lPower = 1;
            rPower = -1;
            fPower = -1;
            bPower = 1;
        }
        else if(gamepad1.left_stick_x < 0)
        {
            lPower = -1;
            rPower = 1;
            fPower = 1;
            bPower = -1;
        }

        if(gamepad1.left_trigger > 0)
        {
            servoLeftPos -= 0.1;
            servoRightPos -= 0.1;
        }else if(gamepad1.right_trigger > 0) {
            servoLeftPos += 0.1;
            servoRightPos += 0.1;
        }

//      CONSTRAIN MOTOR POWERS: -1 to 1 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        if(lPower > 1)
        {
            lPower = 1;
        }
        else if (lPower < -1)
        {
            lPower = -1;
        }

        if(rPower > 1)
        {
            rPower = 1;
        }
        else if (rPower < -1)
        {
            rPower = -1;
        }

        if(fPower > 1)
        {
            fPower = 1;
        }
        else if (fPower < -1)
        {
            fPower = -1;
        }

        if(bPower > 1)
        {
            bPower = 1;
        }
        else if (bPower < -1)
        {
            bPower = -1;
        }

        //BringDown hook = new BringDown();


//      SCALING POWERS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        if(gamepad1.right_bumper)
        {
            lPower *= 0.4;
            rPower *= 0.4;
            fPower *= 0.4;
            bPower *= 0.4;
        }

//      SETTING POWERS AND POSITIONS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

        robot.motorLeft.setPower(lPower);
        robot.motorRight.setPower(rPower);
        robot.motorFront.setPower(fPower);
        robot.motorBack.setPower(bPower);
        robot.motorHook.setPower(hPower);

        robot.servoRight.setPosition(servoRightPos);
        robot.servoLeft.setPosition(servoLeftPos);

//      TELEMETRY
        telemetry.addData("Left Motor Power", gamepad1.left_stick_y);
        telemetry.addData("Right Motor Power", gamepad1.left_stick_y);
        telemetry.addData("Front Motor Power", gamepad1.left_stick_x);
        telemetry.addData("Back Motor Power", gamepad1.left_stick_x);
        telemetry.addData("Hook Motor Power", gamepad2.left_stick_y);

    }

//--------------------------------- FUNCTIONS ----------------------------------------------------

    public double scalePower(double power1)
    {
        double power2;
        if(power1 > 0){
            power2 = Math.pow(Math.abs(power1), 0.6);
        } else if(power1 < 0){
            power2 = -Math.pow(Math.abs(power1), 0.6);
        } else{
            power2 = 0;
        }

        return power2;
    }
}