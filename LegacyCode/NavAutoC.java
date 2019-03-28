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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


//THE KEY BELOW IS FOR USING IMAGE RECOGNITION -PULKIT (Credentials can be found in the viridian gmail account+
//
//
//AT5CUvf/////AAAAGaBn6TlejU79iRr5dpGz0Msa4+WbMquS0c0rHQGMURBOGIxPznOmaavjYRYfWHE/qRnpaHDvKIVV1drOmZguwKjiTVfUzVpkRgxdFzcVDsNBldxzhrcSl+bRKGlNv3zKHDfaOJioTa7uzIN/uKUzdJPX+o5PQQxRPYXBuIvAkASbZ9/MVjq5u3Jltyw3Gz9DCPVgxqcMKILOwv9FpMDMRTcgeRwk7f+pPd8f5FmB8ehr3xiDbPxydmYAkpuqQ6Mx2qiggbSlzl4uTm2JeqOP3hbej+ozcevtHKh9C4S3eKodfDUpKekBfdOuR2aer0FwrWxfAqmdOewy5Tei71lLAOgEzx+vo6OPKpSzbTh1gFzI

//@Autonomous(name="TempAuto", group="Zippo")
//@Disabled
public class NavAutoC extends LinearOpMode {

    /* Declare OpMode members. */
    RuckusHardware robot   = new RuckusHardware();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    //static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // This is for the AndyMark motors //Use 560 for neverest 20, 1120 for neverest 40
    static final double     COUNTS_PER_MOTOR_REV    = 560 ; //These are for the new motors
    static final double     DRIVE_GEAR_REDUCTION    = 25/16;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 90/25.4;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     COUNTS_PER_INCH_LATERAL         = (1120 * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     SERVO_UP                = 0.96;
    static final double     SERVO_DOWN              = 0;
    static final double     SERVO_CLOSED            = 0.2;
    static final double     SERVO_OPEN              = 0.8;

    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor


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

        robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //initialize VuMark
        /*
        relicTrackables.activate();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //print the angles at the beginning of the program
        telemetry.addData("Z", readAngle("z"));
        telemetry.addData("y", readAngle("y"));
        telemetry.addData("x", readAngle("x"));
        telemetry.update();
        */


        //telemetry.addData("Vuforia Status", vuMark);
        telemetry.addData("Z", readAngle("z"));
        telemetry.addData("y", readAngle("y"));
        telemetry.addData("x", readAngle("x"));
        telemetry.update();
        sleep(1500);

        // Main program

        while (opModeIsActive())
        {
            VectorF translation = lastLocation.getTranslation();
            float xPos = translation.get(0)/mmPerInch;
            float yPos = translation.get(1)/mmPerInch;
            float zPos = translation.get(2)/mmPerInch;

            telemetry.addData("Position (inches)","{X, Y, Z} = .1%f, .1%f, .1%f", xPos, yPos, zPos);

            telemetry.addData("Dude this is working good job", "Complete");
            telemetry.update();
        }

    }

    public void moveRobot(double speed, double[] currPos, double[] goalPos, double rightPower)
    {
        /*
            * This is a method to move the angle from one point to another
            * Viridian Robotics October 2018
            *
            * Parameters
            *   -speed: speed at which the robot travels
            *   -currPos: current position of the robot
            *   -goalPos: goal position of the robot
            *   -rightPower: direction that the right tread must move. This indicates the direction
            *                that the robot will turn (left x degrees or right x degrees). If the
            *                value is negative, the right tread will move backwards and the robot
            *                will turn to the right; if the value is positive, the right tread will
            *                move forwards and the robot will turn to the left.
            * First, moveRobot acquires the position it must turn in order to
        */
        double turningAngle = findAngle(currPos, goalPos);
        double treadTravel = turnAngle(16, turningAngle);

        double leftPower = -1 * (rightPower);

        double xPos = currPos[0] - goalPos[0];
        double yPos = currPos[1] - goalPos[1];

        double driveInches = Math.sqrt((yPos*yPos) + (xPos*xPos)); //using pythagorean theorem

        encoderDrive(speed, leftPower*treadTravel, rightPower*treadTravel, 10); //turn to proper heading
        encoderDrive(speed, driveInches, driveInches, 10);



    }

    public static double findAngle(double[] currPos, double[] goalPos)
    {
        double currXPos = currPos[0];
        double currYPos = currPos[1];

        double goalXPos = goalPos[0];
        double goalYPos = goalPos[1];

        double xComp = Math.abs((currXPos-goalXPos));
        double yComp = Math.abs((currYPos-goalYPos));

        double turningAngle = 180 - Math.toDegrees(Math.atan((yComp/xComp)));

        turningAngle = Math.round(turningAngle);

        return turningAngle;
    }

    public static double turnAngle(double robotWidth, double angle)
    {
        //takes the width of the robot and calculates how far each tread needs to move
        //all movements are optimized to use the least area for movement
        double fractionOfCirc = angle/360.0;
        double circumference = 2 * Math.PI * robotWidth;
        double treadTravel = fractionOfCirc * circumference;

        treadTravel = Math.round(treadTravel);

        return treadTravel;
    }
    //drive methods
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
    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
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

}
