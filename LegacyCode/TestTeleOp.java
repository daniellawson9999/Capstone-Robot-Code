package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="TestTeleop", group="Test")
//@Disabled

//THE KEY BELOW IS FOR USING IMAGE RECOGNITION -PULKIT
//AT5CUvf/////AAAAGaBn6TlejU79iRr5dpGz0Msa4+WbMquS0c0rHQGMURBOGIxPznOmaavjYRYfWHE/qRnpaHDvKIVV1drOmZguwKjiTVfUzVpkRgxdFzcVDsNBldxzhrcSl+bRKGlNv3zKHDfaOJioTa7uzIN/uKUzdJPX+o5PQQxRPYXBuIvAkASbZ9/MVjq5u3Jltyw3Gz9DCPVgxqcMKILOwv9FpMDMRTcgeRwk7f+pPd8f5FmB8ehr3xiDbPxydmYAkpuqQ6Mx2qiggbSlzl4uTm2JeqOP3hbej+ozcevtHKh9C4S3eKodfDUpKekBfdOuR2aer0FwrWxfAqmdOewy5Tei71lLAOgEzx+vo6OPKpSzbTh1gFzI

public class TestTeleOp extends OpMode{
    TestHardware robot  = new TestHardware();
    private double lPower = 0;
    private double rPower = 0;
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
        lPower = -gamepad1.right_stick_y;
        rPower = -gamepad1.right_stick_y;
        //latPower =  -(gamepad2.right_trigger - gamepad2.left_trigger);

        double turn = gamepad1.right_stick_x;
        lPower = lPower + turn;
        rPower = rPower - turn;
//      >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

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

        //lPower = scalePower(lPower);
        //rPower = scalePower(rPower);
        //latPower = scalePower(latPower);
        //liftPower = scalePower(liftPower);
        //relicVPower = scalePower(relicVPower);
        //relicPower = scalePower(relicPower);
//      >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

        robot.motorLeft.setPower(lPower);
        //robot.motorRight.setPower(rPower);


//      TELEMETRY
        telemetry.addData("Right-X", gamepad1.right_stick_x);
        telemetry.addData("lefttrig", gamepad1.left_trigger);
        telemetry.addData("righttrig", gamepad1.right_trigger);
        telemetry.addData("Left-Y", gamepad1.left_stick_y);
        telemetry.addData("Right-Y", gamepad1.right_stick_y);
        telemetry.addData("Left-Power", scalePower(lPower));
        telemetry.addData("Right-Power", scalePower(rPower));
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
