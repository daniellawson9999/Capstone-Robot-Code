package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="RuckusTeleop2", group="Zippo")
@Disabled
//THE KEY BELOW IS FOR USING IMAGE RECOGNITION -PULKIT
//AT5CUvf/////AAAAGaBn6TlejU79iRr5dpGz0Msa4+WbMquS0c0rHQGMURBOGIxPznOmaavjYRYfWHE/qRnpaHDvKIVV1drOmZguwKjiTVfUzVpkRgxdFzcVDsNBldxzhrcSl+bRKGlNv3zKHDfaOJioTa7uzIN/uKUzdJPX+o5PQQxRPYXBuIvAkASbZ9/MVjq5u3Jltyw3Gz9DCPVgxqcMKILOwv9FpMDMRTcgeRwk7f+pPd8f5FmB8ehr3xiDbPxydmYAkpuqQ6Mx2qiggbSlzl4uTm2JeqOP3hbej+ozcevtHKh9C4S3eKodfDUpKekBfdOuR2aer0FwrWxfAqmdOewy5Tei71lLAOgEzx+vo6OPKpSzbTh1gFzI

public class RuckusTeleop2 extends OpMode{

    ZippoHardware robot  = new ZippoHardware();

    private double lPower = 0;
    private double rPower = 0;
    private double latPower = 0;
    private double liftPower = 0;
    private double relicPower = 0;
    private double relicVPower = 0;
    private double servoArmPos = 0.96;
    private double servoLeftPos = 0.5;
    private double servoRightPos = 0.5;
    private double servoRelicPos = 0;


    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        //robot.sensorColor.enableLed(false);
        servoLeftPos = .2;
        servoRightPos = .2;

    }

    @Override
    public void loop() {







//      MAIN DRIVING CONTROLS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        lPower = -gamepad1.left_stick_y;
        rPower = -gamepad1.left_stick_y;
        latPower = -gamepad1.left_stick_x;
        //latPower =  -(gamepad2.right_trigger - gamepad2.left_trigger);

        double turn = gamepad1.right_stick_x;
        lPower = lPower + turn;
        rPower = rPower - turn;
//      >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>










//      RELIC ARM CONTROLS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        relicPower = -gamepad2.left_stick_y/3;
        relicVPower = -gamepad2.right_stick_y;

        if(gamepad2.left_bumper)
        {
            if(servoRelicPos > 0) {
                servoRelicPos -= 0.05;
            }
        }else if(gamepad2.right_bumper) {
            if(servoRelicPos < 1) {
                servoRelicPos += 0.05;
            }
        }else if(gamepad2.x){
            if(!(servoRelicPos == 0)){
                servoRelicPos = 0;
            }
        }else if(gamepad2.b){
            if(!(servoRelicPos ==1)){
                servoRelicPos = 1;
            }
        }
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

        if(latPower > 1)
        {
            latPower = 1;
        }
        else if (latPower < -1)
        {
            latPower = -1;
        }

        if(liftPower > 1)
        {
            liftPower = 1;
        }
        else if (liftPower < -1)
        {
            liftPower = -1;
        }

        if(relicPower > 1)
        {
            relicPower = 1;
        }
        else if (relicPower < -1)
        {
            relicPower = -1;
        }

        if(relicVPower > 1)
        {
            relicVPower = 1;
        }
        else if (relicVPower < -1)
        {
            relicVPower = -1;
        }
//      >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>






//      GLYPH ARM CONTROLS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        if(gamepad1.dpad_down)
        {
            liftPower = .5;
        }
        else if(gamepad1.dpad_up)
        {
            liftPower = .5;
        }
        else
        {
            liftPower = 0;
        }

        if(gamepad1.left_trigger > 0)
        {
            servoLeftPos -= 0.1;
            servoRightPos -= 0.1;
        }else if(gamepad1.right_trigger > 0) {
            servoLeftPos += 0.1;
            servoRightPos += 0.1;
        }
    // presets for glyph claw
        if(gamepad1.x){
            servoLeftPos = .8;
            servoRightPos = .8;
        }
        if(gamepad1.b){
            servoLeftPos = .2;
            servoRightPos = .2;

        }
//      >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>




//      CONSTRAIN GLYPH SERVO POSITIONS (FROM 0.2 to 0.8 JUST BECAUSE) <<<<<<<<<<<<<<<<<<<<<

        if(servoLeftPos > .8)
        {
            servoLeftPos = .8;
        }
        else if (servoLeftPos < 0.2)
        {
            servoLeftPos = 0.2;
        }

        if(servoRightPos > .8)
        {
            servoRightPos = .8;
        }
        else if (servoRightPos < 0.2)
        {
            servoRightPos = 0.2;
        }

//      >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>




//      JEWEL HITTER CONTROLS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        if(gamepad1.a)
        {
            if(servoArmPos > 0)
            {
                servoArmPos -= 0.05;
            }
        }
        else if(gamepad1.y)
        {
            if(servoArmPos < 1)
            {
                servoArmPos += 0.05;
            }
        }
//      >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>






//      SCALING POWERS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        if(gamepad1.right_bumper)
        {
            lPower *= 0.3;
            rPower *= 0.3;
            latPower *= 0.3;
            liftPower *= 0.3;
            relicPower *= 0.3;
            relicVPower *= 0.3;
        }

        //lPower = scalePower(lPower);
        //rPower = scalePower(rPower);
        //latPower = scalePower(latPower);
        //liftPower = scalePower(liftPower);
        //relicVPower = scalePower(relicVPower);
        //relicPower = scalePower(relicPower);
//      >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>







//      SETTING POWERS AND POSITIONS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        robot.servoRight.setPosition(servoRightPos);
        robot.servoLeft.setPosition(servoLeftPos);
        robot.servoRelic.setPosition(servoRelicPos);
        robot.servoArm.setPosition(servoArmPos);

        robot.motorLeft.setPower(lPower);
        robot.motorRight.setPower(rPower);
        robot.motorLateral.setPower(-latPower);
        robot.motorPulley.setPower(liftPower);
        robot.motorRelic.setPower(relicPower);
        robot.motorRelicVertical.setPower(relicVPower);
//      >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>



//      TELEMETRY
        telemetry.addData("Right-X", gamepad1.right_stick_x);
        telemetry.addData("lefttrig", gamepad1.left_trigger);
        telemetry.addData("righttrig", gamepad1.right_trigger);
        telemetry.addData("Left-Y", gamepad1.left_stick_y);
        telemetry.addData("Right-Y", gamepad1.right_stick_y);
        telemetry.addData("latPower", scalePower(-latPower));
        telemetry.addData("Left-Power", scalePower(lPower));
        telemetry.addData("Right-Power", scalePower(rPower));
        telemetry.addData("Red", robot.sensorColor.red());
        telemetry.addData("Green", robot.sensorColor.green());
        telemetry.addData("Blue", robot.sensorColor.blue());
        telemetry.addData("ServoArmPosition", servoArmPos);
        telemetry.addData("centimeters", robot.sensorRange.getDistance(DistanceUnit.CM));
        telemetry.addData("Pulley-Power", scalePower(liftPower));
        telemetry.addData("leftClaw-Position", servoLeftPos);
        telemetry.addData("RightClaw-Position", servoRightPos);
        telemetry.addData("RelicClaw-Position", servoRelicPos);
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
