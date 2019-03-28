package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Teleop", group="Zippo")
//@Disabled

//THE KEY BELOW IS FOR USING IMAGE RECOGNITION -PULKIT
//AT5CUvf/////AAAAGaBn6TlejU79iRr5dpGz0Msa4+WbMquS0c0rHQGMURBOGIxPznOmaavjYRYfWHE/qRnpaHDvKIVV1drOmZguwKjiTVfUzVpkRgxdFzcVDsNBldxzhrcSl+bRKGlNv3zKHDfaOJioTa7uzIN/uKUzdJPX+o5PQQxRPYXBuIvAkASbZ9/MVjq5u3Jltyw3Gz9DCPVgxqcMKILOwv9FpMDMRTcgeRwk7f+pPd8f5FmB8ehr3xiDbPxydmYAkpuqQ6Mx2qiggbSlzl4uTm2JeqOP3hbej+ozcevtHKh9C4S3eKodfDUpKekBfdOuR2aer0FwrWxfAqmdOewy5Tei71lLAOgEzx+vo6OPKpSzbTh1gFzI

public class ZoTeleOp extends OpMode{

    ZoHardware robot  = new ZoHardware();

    // Create variables for motor power
    private double lPower = 0; //left wheel
    private double rPower = 0; //right wheel
    private double fPower = 0; //front power
    private double bPower = 0; //back power
    private double hPower = 0; //hook power
    private double aPowerExt = 0; //arm extension power
    private double aPowerTIlt = 0; //intake bocx tilt power
    private double boxLiftPower = 0; // lifty boi power


    private double boxPower = 0; //boxPower
    private double boxTilt = 0;


    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        //robot.sensorColor.enableLed(false);
        double time =  System.currentTimeMillis();
        while(time + 500 > System.currentTimeMillis())
        {
            robot.motorLiftBucket.setPower(70);
        }
        robot.motorLiftBucket.setPower(0);
    }

    @Override
    public void loop() {

//      MAIN DRIVING CONTROLS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//        lPower = gamepad1.right_stick_y;
//        rPower = gamepad1.right_stick_y;
//        fPower = -gamepad1.right_stick_x;
//        bPower = -gamepad1.right_stick_x;
        if (gamepad1.right_stick_y != 0)
        {
            lPower = gamepad1.right_stick_y;
            rPower = gamepad1.right_stick_y;
        }

        else if (gamepad2.right_stick_y != 0)
        {
            lPower = gamepad2.right_stick_y;
            rPower = gamepad2.right_stick_y;
        }
        else
        {
            lPower = 0;
            rPower = 0;
        }
        if (gamepad1.right_stick_x != 0)
        {
            fPower = -gamepad1.right_stick_x;
            bPower = -gamepad1.right_stick_x;
        }
        else if (gamepad2.right_stick_x != 0)
        {
            fPower = -gamepad2.right_stick_x;
            bPower = -gamepad2.right_stick_x;
        }
        else
        {
            fPower = 0;
            bPower = 0;
        }


        if(gamepad1.dpad_up || gamepad2.dpad_up)
        {
            hPower = -1;
        }
        else if(gamepad1.dpad_down || gamepad2.dpad_down)
        {
            hPower = 1;
        }
        else
        {
            hPower = 0;
        }

        //rotate
        if(gamepad1.left_stick_x > 0 || gamepad2.left_stick_x > 0)
        {
            lPower = -1; //lpower used to be 1
            rPower = 1; //rpower used to be -1
            fPower = -1;
            bPower = 1;
        }
        else if(gamepad1.left_stick_x < 0 || gamepad2.left_stick_x < 0)
        {
            lPower = 1; //lpower used to be -1
            rPower = -1; //used to be 1
            fPower = 1;
            bPower = -1;
        }
        double tiltPower = .5;
        if(gamepad1.left_bumper || gamepad2.left_bumper)
        {
            aPowerTIlt = tiltPower;
        }

        else if (gamepad1.right_bumper || gamepad2.right_bumper)
        {
            aPowerTIlt = -tiltPower;
        }
        else
        {
            aPowerTIlt =0;
        }

        if(gamepad1.left_trigger > 0 || gamepad2.left_trigger > 0)
        {
            aPowerExt = 1;
        }

        else if (gamepad1.right_trigger > 0 || gamepad2.right_trigger > 0)
        {
            aPowerExt = -1;
        }
        else
        {
            aPowerExt =0;
        }

        if (gamepad1.dpad_left || gamepad2.dpad_left)
        {
            boxLiftPower = 1;
        }
        else if (gamepad1.dpad_right || gamepad2.dpad_right)
        {
            boxLiftPower = -1;
        }
        else
        {
            boxLiftPower = 0;
        }
        boxTilt = 0;
        if (gamepad1.x || gamepad2.x)
        {
           boxTilt = -1;
        }
        if(gamepad1.y || gamepad2.y) {
            boxTilt = 1;
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
        if((gamepad1.a && boxPower >= 0) ||(gamepad2.a && boxPower >= 0))
        {
            boxPower = -1;
        }
        else if((gamepad1.a && boxPower < 0) || (gamepad2.a && boxPower < 0))
        {
            boxPower = 1;
        }
        if((gamepad1.b && boxPower != 0) || (gamepad2.b && boxPower != 0))
        {
            boxPower = 0;
        }
//      SETTING POWERS AND POSITIONS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

        robot.motorLeft.setPower(fPower);
        robot.motorRight.setPower(bPower);
        robot.motorFront.setPower(rPower);
        robot.motorBack.setPower(lPower);
        robot.motorHook.setPower(hPower);
        robot.motorArmExt.setPower(aPowerExt);
        robot.motorArmTilt.setPower(aPowerTIlt);
        robot.motorLiftBucket.setPower(boxLiftPower);
        robot.servoIntake.setPower(boxPower);
        robot.servoBox.setPower(boxTilt);


//      TELEMETRY
        telemetry.addData("Left Motor Power", gamepad1.left_stick_y);
        telemetry.addData("Right Motor Power", gamepad1.left_stick_y);
        telemetry.addData("Front Motor Power", gamepad1.left_stick_x);
        telemetry.addData("Back Motor Power", gamepad1.left_stick_x);
        telemetry.addData("Hook Motor Power", gamepad2.left_stick_y);
        telemetry.addData("Arm Tilt Motor Power", aPowerTIlt);
        telemetry.addData("Arm Ext Motor Power", aPowerExt);
        telemetry.addData("Box intake Power", boxPower);
        telemetry.addData("Right Distance", robot.sensorRangeR.getDistance(DistanceUnit.INCH));
        telemetry.addData("Left Distance", robot.sensorRangeL.getDistance(DistanceUnit.INCH));
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