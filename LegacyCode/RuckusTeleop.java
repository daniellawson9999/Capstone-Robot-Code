package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//@TeleOp(name="RuckusTeleop", group="Zippo")

//THE KEY BELOW IS FOR USING IMAGE RECOGNITION -PULKIT
//AT5CUvf/////AAAAGaBn6TlejU79iRr5dpGz0Msa4+WbMquS0c0rHQGMURBOGIxPznOmaavjYRYfWHE/qRnpaHDvKIVV1drOmZguwKjiTVfUzVpkRgxdFzcVDsNBldxzhrcSl+bRKGlNv3zKHDfaOJioTa7uzIN/uKUzdJPX+o5PQQxRPYXBuIvAkASbZ9/MVjq5u3Jltyw3Gz9DCPVgxqcMKILOwv9FpMDMRTcgeRwk7f+pPd8f5FmB8ehr3xiDbPxydmYAkpuqQ6Mx2qiggbSlzl4uTm2JeqOP3hbej+ozcevtHKh9C4S3eKodfDUpKekBfdOuR2aer0FwrWxfAqmdOewy5Tei71lLAOgEzx+vo6OPKpSzbTh1gFzI

public class RuckusTeleop extends OpMode{

    RuckusHardware robot  = new RuckusHardware();

    // Create variables for motor power
    private double lPower = 0;
    private double rPower = 0;
    private double fPower = 0;
    private double bPower = 0;

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {

//      MAIN DRIVING CONTROLS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        lPower = -gamepad1.left_stick_y;
        rPower = -gamepad1.right_stick_y;
        fPower = -gamepad2.left_stick_y;
        bPower = -gamepad2.right_stick_y;


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
        else if (lPower < -1)
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


//      SCALING POWERS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        if(gamepad1.right_bumper)
        {
            lPower *= 0.3;
            rPower *= 0.3;
        }
        if(gamepad2.right_bumper)
        {
            bPower *= 0.3;
        }
        if(gamepad2.left_bumper)
        {
            fPower *= 0.3;
        }


//      SETTING POWERS AND POSITIONS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

        robot.motorLeft.setPower(lPower);
        robot.motorRight.setPower(rPower);


//      TELEMETRY
        telemetry.addData("Left Motor Power", lPower);
        telemetry.addData("Right Motor Power", rPower);
        telemetry.addData("Back Motor Power", bPower);
        telemetry.addData("Front Motor Power", fPower);
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