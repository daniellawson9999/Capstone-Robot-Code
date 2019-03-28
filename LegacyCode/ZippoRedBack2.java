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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


//THE KEY BELOW IS FOR USING IMAGE RECOGNITION -PULKIT (Credentials can be found in the viridian gmail account+
//
//
//AT5CUvf/////AAAAGaBn6TlejU79iRr5dpGz0Msa4+WbMquS0c0rHQGMURBOGIxPznOmaavjYRYfWHE/qRnpaHDvKIVV1drOmZguwKjiTVfUzVpkRgxdFzcVDsNBldxzhrcSl+bRKGlNv3zKHDfaOJioTa7uzIN/uKUzdJPX+o5PQQxRPYXBuIvAkASbZ9/MVjq5u3Jltyw3Gz9DCPVgxqcMKILOwv9FpMDMRTcgeRwk7f+pPd8f5FmB8ehr3xiDbPxydmYAkpuqQ6Mx2qiggbSlzl4uTm2JeqOP3hbej+ozcevtHKh9C4S3eKodfDUpKekBfdOuR2aer0FwrWxfAqmdOewy5Tei71lLAOgEzx+vo6OPKpSzbTh1gFzI

@Autonomous(name="RedBack2", group="Zippo")
@Disabled
public class ZippoRedBack2 extends LinearOpMode {

    /* Declare OpMode members. */
    ZippoHardware robot   = new ZippoHardware();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    //static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // This is for the AndyMark motors //Use 560 for neverest 20, 1120 for neverest 40
    static final double     COUNTS_PER_MOTOR_REV    = 560 ; //These are for the new motors
    static final double     DRIVE_GEAR_REDUCTION    = 25/16;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 90/25.4;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     COUNTS_PER_INCH_LATERAL         = (1120 * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     SERVO_UP                = 0.96;
    static final double     SERVO_DOWN              = 0;
    static final double     SERVO_CLOSED            = 0.8;
    static final double     SERVO_OPEN              = 0.2;

    BNO055IMU imu;



    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters1 = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters1.vuforiaLicenseKey = "AT5CUvf/////AAAAGaBn6TlejU79iRr5dpGz0Msa4+WbMquS0c0rHQGMURBOGIxPznOmaavjYRYfWHE/qRnpaHDvKIVV1drOmZguwKjiTVfUzVpkRgxdFzcVDsNBldxzhrcSl+bRKGlNv3zKHDfaOJioTa7uzIN/uKUzdJPX+o5PQQxRPYXBuIvAkASbZ9/MVjq5u3Jltyw3Gz9DCPVgxqcMKILOwv9FpMDMRTcgeRwk7f+pPd8f5FmB8ehr3xiDbPxydmYAkpuqQ6Mx2qiggbSlzl4uTm2JeqOP3hbej+ozcevtHKh9C4S3eKodfDUpKekBfdOuR2aer0FwrWxfAqmdOewy5Tei71lLAOgEzx+vo6OPKpSzbTh1gFzI";
        parameters1.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters1);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLateral.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorLateral.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.servoArm.setPosition(0.96);
        //robot.servoLeft.setPosition(0.2); //l1 r 0
        //robot.servoRight.setPosition(0.8);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //initialize VuMark
        relicTrackables.activate();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //print the angles at the beginning of the program
        telemetry.addData("Z", readAngle("z"));
        telemetry.addData("y", readAngle("y"));
        telemetry.addData("x", readAngle("x"));
        telemetry.update();

        robot.servoLeft.setPosition(SERVO_CLOSED); //l1 r 0
        robot.servoRight.setPosition(SERVO_CLOSED);

        //forward and then turn
        //change angle
        robot.motorPulley.setPower(1);
        sleep(250);
        robot.motorPulley.setPower(0);
        sleep(1000);

        robot.servoArm.setPosition(SERVO_DOWN);
        sleep(1000);

        //Jewel-hitting algorithm goes here
        int ang = 20; //how many degrees the bot will turn to hit a jewel
        if(robot.sensorColor.red() < robot.sensorColor.blue())
        {
            gyroDrive(ang, "z", 0.3, 4);
            sleep(500);
            //encoderDrive(0.2,  3,  3, 5.0);
            robot.servoArm.setPosition(SERVO_UP);
            sleep(500);
            runtime.reset();

            while(opModeIsActive() && vuMark == RelicRecoveryVuMark.UNKNOWN && runtime.seconds() < 3)
            {
                telemetry.addData("Vuforia Status", "cannot find picture");
                telemetry.update();
                vuMark = RelicRecoveryVuMark.from(relicTemplate);
            }
            gyroDrive(0, "z", 0.3, 4);
        }
        else
        {
            gyroDrive(-ang, "z", 0.3, 4);
            sleep(500);
            //encoderDrive(0.2,  -3,  -3, 5.0);
            robot.servoArm.setPosition(SERVO_UP);
            sleep(500);
            gyroDrive(0, "z", 0.3, 4);
            sleep(500);
            gyroDrive(ang, "z", 0.3, 4);
            sleep(500);
            runtime.reset();
            while(opModeIsActive() && vuMark == RelicRecoveryVuMark.UNKNOWN && runtime.seconds() < 3)
            {
                telemetry.addData("Vuforia Status", "cannot find picture");
                telemetry.update();
                vuMark = RelicRecoveryVuMark.from(relicTemplate);
            }
            gyroDrive(0, "z", 0.3, 4);
            sleep(500);
        }
        telemetry.addData("Vuforia Status", vuMark);
        telemetry.update();
        sleep(500);

        encoderDrive(0.3, 10, 10, 5);
        gyroDrive(0.9, "y", -0.3, 5);
        gyroDrive(0, "z", 0.3, 5);
        encoderDrive(0.3, 10, 10, 5);

        double rightColumn = -14;
        double centerColumn = -18;
        double leftColumn = -15;

        if(vuMark == RelicRecoveryVuMark.RIGHT){
            //Score into left column
            gyroDrive(-135, "z", 0.2, 5);
            encoderDrive(0.2, rightColumn, rightColumn, 5);
        }/*else if(vuMark == RelicRecoveryVuMark.CENTER){
            //Score into center column
            gyroDrive(-160, "z", 0.2, 5);
            encoderLateralDrive(0.4, 10,5);
            encoderDrive(0.2, centerColumn, centerColumn, 5);
        }*/else if(vuMark == RelicRecoveryVuMark.LEFT){
            //Score into right column

            gyroDrive(-170, "z", 0.2, 5);
            encoderLateralDrive(0.4, 17,5);
            gyroDrive(-150, "z", 0.2, 5);
            encoderDrive(0.2, leftColumn, leftColumn, 5);

        }else{
            gyroDrive(-160, "z", 0.2, 5);
            encoderLateralDrive(0.4, 10,5);
            encoderDrive(0.2, centerColumn, centerColumn, 5);
        }

        robot.servoLeft.setPosition(SERVO_OPEN); //l1 r 0
        robot.servoRight.setPosition(SERVO_OPEN);

        encoderDrive(0.2, 3, 3, 5);

        telemetry.addData("Path", "Complete");
        telemetry.update();

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
            telemetry.addData("angle", angle);
            telemetry.addData("error", error);
            telemetry.update();
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
//        telemetry.addData("we are out", "");
//        telemetry.update();
//        sleep(5000);


    }

    public void encoderDrive(double speed,
                             double leftInches,
                             double rightInches,
                             double timeoutS) {
        leftInches = -leftInches;
        rightInches = -rightInches;
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {
            newLeftTarget = robot.motorLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.motorRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.motorLeft.setTargetPosition(newLeftTarget);
            robot.motorRight.setTargetPosition(newRightTarget);
            robot.motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            robot.motorLeft.setPower(Math.abs(speed));
            robot.motorRight.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.motorLeft.isBusy() && robot.motorRight.isBusy())) {
                telemetry.update();
            }
            telemetry.addData("STOPPED", "ROBOT HAS STOPPED");

            robot.motorLeft.setPower(0);
            robot.motorRight.setPower(0);

            robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
    public void encoderLateralDrive(double speed,
                                    double lateralInches,
                                    double timeoutS) {
        lateralInches = -lateralInches;
        int newLateralTarget;

        if (opModeIsActive()) {
            newLateralTarget = robot.motorLateral.getCurrentPosition() + (int)(lateralInches * COUNTS_PER_INCH_LATERAL);
            robot.motorLateral.setTargetPosition(newLateralTarget);
            robot.motorLateral.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            robot.motorLateral.setPower(Math.abs(speed));
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.motorLateral.isBusy())) {
                telemetry.update();
            }
            telemetry.addData("STOPPED", "ROBOT HAS STOPPED");

            robot.motorLateral.setPower(0);

            robot.motorLateral.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public double angleWrap (double angle){
        double offset;
        if(angle > 180){
            offset = angle - 180;
            angle = -180 + offset;
        }else if(angle < -180){
            offset = angle + 180;
            angle = 180 + offset;
        }
        return angle;
    }

    public double pidMultiplier(double error){
        //equation for power multiplier is x/sqrt(x^2 + C)
        int C = 200;
        return Math.abs(error/Math.sqrt((error * error) + C));
    }

    public void driveToFlap(){
        runtime.reset();
        double distance = robot.sensorRange.getDistance(DistanceUnit.CM);
        while(opModeIsActive() && !(distance < 20) && runtime.seconds() < 5){
            distance = robot.sensorRange.getDistance(DistanceUnit.CM);
            normalDrive(-0.1, -0.1);
        }
        normalDrive(0, 0);

    }
}
