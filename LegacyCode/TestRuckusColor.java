package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="ColorTest", group="Zippo")
//@Disabled

//THE KEY BELOW IS FOR USING IMAGE RECOGNITION -PULKIT
//AT5CUvf/////AAAAGaBn6TlejU79iRr5dpGz0Msa4+WbMquS0c0rHQGMURBOGIxPznOmaavjYRYfWHE/qRnpaHDvKIVV1drOmZguwKjiTVfUzVpkRgxdFzcVDsNBldxzhrcSl+bRKGlNv3zKHDfaOJioTa7uzIN/uKUzdJPX+o5PQQxRPYXBuIvAkASbZ9/MVjq5u3Jltyw3Gz9DCPVgxqcMKILOwv9FpMDMRTcgeRwk7f+pPd8f5FmB8ehr3xiDbPxydmYAkpuqQ6Mx2qiggbSlzl4uTm2JeqOP3hbej+ozcevtHKh9C4S3eKodfDUpKekBfdOuR2aer0FwrWxfAqmdOewy5Tei71lLAOgEzx+vo6OPKpSzbTh1gFzI

public class TestRuckusColor extends OpMode {

    TestRuckusHardware robot = new TestRuckusHardware();

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.servoLeft.setPosition(1.0);
        robot.servoRight.setPosition(1.0);
    }

    @Override
    public void loop() {

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        telemetry.addData("Red Left: ", robot.sensorColorL.red());
        telemetry.addData("Green Left: ", robot.sensorColorL.green());
        telemetry.addData("Blue Left: ", robot.sensorColorL.blue());

        double whiteValue = Math.pow(580.7 - robot.sensorColorL.red(), 4) + Math.pow(612.2 - robot.sensorColorL.green(), 4)
                + Math.pow(554.5 - robot.sensorColorL.blue(), 4)*1.2;
        double yellowValue = Math.pow(180.1 - robot.sensorColorL.red(), 4) + Math.pow(129.8 - robot.sensorColorL.green(), 4)
                + Math.pow(78.8 - robot.sensorColorL.blue(), 4)*1.2;

        telemetry.addData("Yellow Value", yellowValue);
        telemetry.addData("White Value", whiteValue);

        if(whiteValue < yellowValue)
        {
            telemetry.addData("ITS WHITE", whiteValue);
            telemetry.update();
        }
        else{
            telemetry.addData("ITS YELLOW", yellowValue);
            telemetry.update();
        }

    }

}
