/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package edu.elon.robotics;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

/**
 * Test program for multiple sensors.
 *
 * @author Jochen Fischer
 * */

@TeleOp(name="Drive w/4 Sensors", group="Test")
//@Disabled
public class DriveWith4Sensors extends LinearOpMode {

    // Declare OpMode members
    HardwareDriveBotWithIMU robot = new HardwareDriveBotWithIMU();
    String TAG = "SensorDrive";

    @Override
    public void runOpMode() {

        // Initialize the hardware and calibrate the IMU
        telemetry.addData("Calibrating", "please wait...");
        telemetry.update();

        robot.init(hardwareMap);
        Log.d(TAG, "after robot.init");

        // get a first reading from the IMU:
        Orientation angles  = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Acceleration gravity = robot.imu.getGravity();
        double currentHeading = angles.firstAngle;
        double lastHeading = currentHeading;
        double absoluteHeading = currentHeading;  // including one or several turns:
        int nTurns = 0;

        // Send telemetry message to signify robot waiting;
        telemetry.addData("IMU calibrated", "%s", robot.calibratedIMU ? "YES" : "NO");
        telemetry.addData("Initialized", "Press START now...");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // slow movement is the default, faster when left bumper is pressed:
            double factor = 0.2;
            if( gamepad1.left_bumper ) factor = 0.5;

            // POV drive (left stick forward, right stick turn):
            double speed = -gamepad1.left_stick_y * factor;
            double turn  = gamepad1.right_stick_x * factor;

            double left  = Range.clip(speed+turn, -1.0, 1.0);
            double right = Range.clip(speed-turn, -1.0, 1.0);

            robot.leftMotor.setPower(left);
            robot.rightMotor.setPower(right);

            telemetry.addLine()
                .addData("pwrL", "%.2f", left)
                .addData("pwrR", "%.2f", right);

            // read the motor encoders:
            int encR = robot.rightMotor.getCurrentPosition();
            int encL = robot.leftMotor.getCurrentPosition();

            telemetry.addLine()
                    .addData("encL", "%d", encL)
                    .addData("encR", "%d", encR);

            //-----------------------------------------------
            // move the servo with the Y and A buttons:
            //-----------------------------------------------
            //if     ( gamepad1.y ) robot.frontServo.setPosition(0.8);
            //else if( gamepad1.a ) robot.frontServo.setPosition(0.2);
            //else                  robot.frontServo.setPosition(0.5);

            //------------------------------
            // read the touch sensor:
            //------------------------------
            telemetry.addLine().addData("touch", robot.touchSensor.isPressed() ? "ON" : "off");

            //------------------------------
            // read the ultrasonic sensor:
            //------------------------------
            telemetry.addLine().addData("Lego US (cm)", robot.ultraSonic.getUltrasonicLevel());

            //-----------------------------------------------
            // Read the color sensor
            //-----------------------------------------------
            int A = robot.colorSensor.alpha();
            int R = robot.colorSensor.red();
            int G = robot.colorSensor.green();
            int B = robot.colorSensor.blue();
            telemetry.addLine()
                    .addData("A", "%3d", A)
                    .addData("R", "%3d", R)
                    .addData("G", "%3d", G)
                    .addData("B", "%3d", B);

            //------------------------------
            // Read all data from the IMU
            //------------------------------
            angles  = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity = robot.imu.getGravity();

            telemetry.addLine()
                    .addData("status", robot.imu.getSystemStatus().toShortString())
                    .addData("calib",  robot.imu.getCalibrationStatus().toString());

            telemetry.addLine()
                    .addData("heading", robot.formatAngle(angles.angleUnit, angles.firstAngle))
                    .addData("pitch",   robot.formatAngle(angles.angleUnit, angles.secondAngle))
                    .addData("roll",    robot.formatAngle(angles.angleUnit, angles.thirdAngle));

            telemetry.addLine()
                    .addData("grvty", gravity.toString())
                    .addData("mag",   String.format(Locale.getDefault(), "%.3f",
                                    Math.sqrt(gravity.xAccel*gravity.xAccel
                                            + gravity.yAccel*gravity.yAccel
                                            + gravity.zAccel*gravity.zAccel)));

            //-------------------------------------------------------
            // keep track of the heading including multiple turns:
            //-------------------------------------------------------
            currentHeading = angles.firstAngle;

            // Keep track of the turns if there is a jump from 180 deg to -180 deg.
            // This happens on left turns where the angle increases steadily,
            // but jumps from +180 to -180.
            if( currentHeading-lastHeading < -180.0 ) nTurns++;

            // Opposite jump from -180 to +180 happens on right turns:
            if( currentHeading-lastHeading > +180.0 ) nTurns--;

            // get the absolute heading, including any turns:
            absoluteHeading  = currentHeading + nTurns * 360.0;
            telemetry.addData("absolute heading", String.format("%7.1f", absoluteHeading));
            lastHeading = currentHeading;  // save heading for next loop

            //----------------------------------------------
            // push all data to the driver station phone:
            //----------------------------------------------
            telemetry.update();

            // Pause for metronome tick. 50ms each cycle = update 20 times a second.
            robot.waitForTick(50);

            // or hand control to the other processes for a while:
            // idle();
        }
    }
}
