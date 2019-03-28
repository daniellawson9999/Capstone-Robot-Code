/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="BigAngleTestCrater", group="Zippo")
//@Disabled
public class BigAngleTestCrater extends LinearOpMode {

    /* Declare OpMode members. */
    TestRuckusHardware         robot   = new TestRuckusHardware();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 288 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.5 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.4 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 1.0;
    static final double     TURN_SPEED              = 0.5;
    static final double COUNTS_PER_MOTOR = 1120; //for the hook motor
    static final double COUNTS_PER_INCH_HOOK = (COUNTS_PER_MOTOR*2*3.1415);
    BNO055IMU imu;

    @Override
    public void runOpMode() {
        String xyz = "z";
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        DistanceSensor sensorRange = hardwareMap.get(DistanceSensor.class, "sensorRange");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Send telemetry message to signify robot waiting;
        /*telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();*/

        //side motors
        robot.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //lateral motors
        robot.motorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //hook motor
        robot.motorHook.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorHook.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        /*telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.motorLeft.getCurrentPosition(),
                robot.motorRight.getCurrentPosition());
        telemetry.update();*/

        // Wait for the game to start (driver presses PLAY)
        robot.servoMark.setPosition(0);
        waitForStart();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        hookEncoder(3, -.81, 5);

        sleep(1000);

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        latEncoderDrive(.6,  -4,  -4, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        robot.servoRight.setPosition(0);
        robot.servoLeft.setPosition(0);
        sleep(500);

        //lateral 36"
        encoderDrive(.7, -24, -24, 5);
        sleep(250);
        /*
        latEncoderDrive(.6,8.5,8.5,4);
        sleep(500);

        double whiteValueLeft = Math.pow(580.7 - robot.sensorColorL.red(), 4) + Math.pow(612.2 - robot.sensorColorL.green(), 4)
                + Math.pow(554.5 - robot.sensorColorL.blue(), 4);
        double yellowValueLeft = Math.pow(180.1 - robot.sensorColorL.red(), 4) + Math.pow(129.8 - robot.sensorColorL.green(), 4)
                + Math.pow(78.8 - robot.sensorColorL.blue(), 4);
        double whiteValueRight = Math.pow(580.7 - robot.sensorColorR.red(), 4) + Math.pow(612.2 - robot.sensorColorR.green(), 4)
                + Math.pow(554.5 - robot.sensorColorR.blue(), 4);
        double yellowValueRight = Math.pow(180.1 - robot.sensorColorR.red(), 4) + Math.pow(129.8 - robot.sensorColorR.green(), 4)
                + Math.pow(78.8 - robot.sensorColorR.blue(), 4);

        encoderDrive(.6,-10,-10,5);
        sleep(500);
        encoderDrive(.6,10,10,5);
        sleep(500);
        */
        /*
        if(yellowValueLeft < whiteValueLeft)
        {
            sleep(500);
            encoderDrive(.6,-10,-10,5);
            sleep(500);
            encoderDrive(.6,10,10,5);
        }
        else if(yellowValueRight < whiteValueRight){
            latEncoderDrive(.6,5,5,3);
            encoderDrive(.6,-7,-7,3);
            sleep(500);
            gyroDrive(45,xyz,.3,3);
            encoderDrive(.6,-3,-3,3);
        }
        else{
            latEncoderDrive(.6,-10,-10,4);
            sleep(300);
            encoderDrive(.6,-12,-12,4);
            gyroDrive(-45,xyz,.3,3);
            sleep(300);
            encoderDrive(.6,-3,-3,3);
        }*/

        /*telemetry.addData("degrees","0");
        telemetry.update();
        gyroDrive(0,xyz,.2,5);
        telemetry.addData("degrees","90");
        telemetry.update();
        gyroDrive(90,xyz,.2,5);
        telemetry.addData("degrees","180");
        telemetry.update();
        gyroDrive(180,xyz,.2,5);
        telemetry.addData("degrees","-90");
        telemetry.update();
        gyroDrive(-90,xyz,.2,5);
        telemetry.addData("degrees","0");
        telemetry.update();
        gyroDrive(0,xyz,.2,5);*/
        //-135
      latEncoderDrive(.7,-24,-24,5);
      sleep(500);
      gyroDrive(-45,xyz,.3,5);
      robot.servoLeft.setPosition(0);
      robot.servoRight.setPosition(0);
      sleep(250);
      double tdistance = 9;
      double cdistance = sensorRange.getDistance(DistanceUnit.INCH);
      latEncoderDrive(DRIVE_SPEED,-(cdistance-tdistance),-(cdistance-tdistance),3);
      sleep(500);
      gyroDrive(90,xyz,.3,3);
      sleep(500);
      gyroDrive(126,xyz,.3,3);
      sleep(500);
      encoderDrive(.6,-65,-65,6);
      //release servo
        robot.servoMark.setPosition(1);
        sleep(500);
        encoderDrive(.6,68,68,6);
        //maybe move this line to higher>
        robot.servoMark.setPosition(0);
    }

    public static double turnDistance(double axleLength, double angle)
    {
        //takes the width of the robot and calculates how far each tread needs to move
        //all movements are optimized to use the least area for movement
        double fractionOfCirc = angle/360.0;
        double circumference = 2 * Math.PI * axleLength;
        double wheelTravel = fractionOfCirc * circumference;

        wheelTravel = Math.round(wheelTravel);

        return wheelTravel;
    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.motorLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.motorRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.motorLeft.setTargetPosition(newLeftTarget);
            robot.motorRight.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.motorLeft.setPower(Math.abs(speed));
            robot.motorRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.motorLeft.isBusy() && robot.motorRight.isBusy())) {

                /*Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
               telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.motorLeft.getCurrentPosition(),
                        robot.motorRight.getCurrentPosition());
               telemetry.update();*/
            }

            // Stop all motion;
            robot.motorLeft.setPower(0);
            robot.motorRight.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void latEncoderDrive(double speed, double frontInches, double backInches, double timeoutS) {
        int newFrontTarget;
        int newBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontTarget = robot.motorFront.getCurrentPosition() + (int)(frontInches * COUNTS_PER_INCH);
            newBackTarget = robot.motorBack.getCurrentPosition() + (int)(backInches * COUNTS_PER_INCH);
            robot.motorFront.setTargetPosition(newFrontTarget);
            robot.motorBack.setTargetPosition(newBackTarget);

            // Turn On RUN_TO_POSITION
            robot.motorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.motorFront.setPower(Math.abs(speed));
            robot.motorBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.motorFront.isBusy() && robot.motorBack.isBusy())) {

                // Display it for the driver.
               /* telemetry.addData("Path1",  "Running to %7d :%7d", newFrontTarget,  newBackTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.motorFront.getCurrentPosition(),
                        robot.motorBack.getCurrentPosition());
                telemetry.update();*/
            }

            // Stop all motion;
            robot.motorFront.setPower(0);
            robot.motorBack.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.motorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void hookEncoder(double speed, double inches, double timeoutS) {
        int newHookTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newHookTarget = robot.motorHook.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH_HOOK);
            robot.motorHook.setTargetPosition(newHookTarget);

            // Turn On RUN_TO_POSITION
            robot.motorHook.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.motorHook.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.motorHook.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d", newHookTarget);
                telemetry.addData("Path2",  "Running at %7d",
                        robot.motorHook.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.motorHook.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.motorHook.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    public void normalDrive(double lpower, double rpower) {

        if (opModeIsActive()) {
            robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorLeft.setPower(lpower);
            robot.motorRight.setPower(rpower);
        }
    }
    public void gyroDrive(double target, String xyz, double power, double timeoutS)
    {
        //Write code to correct to a target position (NOT FINISHED)
        runtime.reset();
        double angle = readAngle(xyz); //variable for gyro correction around z axis
        double error = angle-target;
        double powerScaled = power*pidMultiplier(error);
        do{
            angle = readAngle(xyz);
            error = angle - target;
            powerScaled = power*pidMultiplier(error);
            /*telemetry.addData("error", error);
            telemetry.update();*/
            if(error > 0)
            {
                if(xyz.equals("z")) {
                    normalDrive(powerScaled, -powerScaled);
                }
                if(xyz.equals("y"))
                {
                    if (opModeIsActive()) {
                        robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.motorLeft.setPower(powerScaled);
                        robot.motorRight.setPower(powerScaled);
                    }
                }
            }
            else if(error < 0)
            {
                if(xyz.equals("z")) {
                    normalDrive(-powerScaled, powerScaled);
                }
                if(xyz.equals("y"))
                {
                    if (opModeIsActive()) {
                        robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.motorLeft.setPower(powerScaled);
                        robot.motorRight.setPower(powerScaled);
                    }
                }
            }
        }while(opModeIsActive() && ((error >1 ) || (error < -1)) && (runtime.seconds() < timeoutS));
        normalDrive(0,0);
    }
    public double pidMultiplier(double error){
        //equation for power multiplier is x/sqrt(x^2 + C)
        int C = 200;
        return Math.abs(error/Math.sqrt((error * error) + C));
    }
    public double readAngle(String xyz)
    {
        Orientation angles;
        Acceleration gravity;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if(xyz.equals("x")){
            return angles.thirdAngle;
        }else if(xyz.equals("y")){
            return angles.secondAngle;
        }else if(xyz.equals("z")){
            return angles.firstAngle;
        }else{
            return 0;
        }
    }

}
