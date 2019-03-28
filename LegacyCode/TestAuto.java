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

@Autonomous(name="TestAuto", group="Test")
@Disabled
public class TestAuto extends LinearOpMode {

    /* Declare OpMode members. */
    TestHardware robot = new TestHardware();   // Use a Pushbot's hardware
    @Override
    public void runOpMode() {
       /* robot.init(hardwareMap);
        waitForStart();
        // Drive in a square
        for(int i = 0; i < 4; i++)
        {
            // Go forward
            robot.motorLeft.setPower(.3);
            robot.motorRight.setPower(.3);
            sleep(2000);
            // Turn 90 degrees
            robot.motorLeft.setPower(-.3);
            robot.motorRight.setPower(.3);
            // Previously 600 ms
            sleep(900);
        }
        robot.motorLeft.setPower(0);
        robot.motorRight.setPower(0);
        sleep(600);
        for (int i = 0; i < 3; i++) {
            robot.motorLeft.setPower(.3);
            robot.motorRight.setPower(.3);
            sleep(1500);
            robot.motorLeft.setPower(.3);
            robot.motorRight.setPower(-.3);
            sleep(1550);
        }
        robot.motorLeft.setPower(0);
        robot.motorRight.setPower(0);
        */
    }

}

