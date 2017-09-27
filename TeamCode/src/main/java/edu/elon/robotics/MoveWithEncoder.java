package edu.elon.robotics;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Moves the robot with encoders.
 *
 * @author Jochen Fischer
 * @version 1
 */

@Autonomous(name="Move With Encoders V1", group="Elon")
//@Disabled
public class MoveWithEncoder extends LinearOpMode {

    // declare robot class:
    HardwareDriveBot robot = new HardwareDriveBot();

    //logging tag
    private static final String TAG = "EncoderTest";

    @Override
    public void runOpMode() {

        // initialize the robot:
        robot.init(hardwareMap);

        // wait for the user to press the START button on the DS phone:
        waitForStart();

        //----------------------------------------
        // move the robot by a given distance:
        //----------------------------------------

        // move the robot by 24 inches:
        double inches = 24;
        double speed = 0.3;

        moveRobot(speed, 72.0);
        moveRobot(speed, -24.0);

        // or:
        moveRobot(-speed, 24);
        turnRobot(speed, 90);


        //----------------------------------------
        // turn the robot by a given angle:
        //----------------------------------------

        robot.resetEncoders();


        double angle = 90.0;
        // TODO: WARNING: NO MAGIC NUMBERS, instead, do the math
        int encoderTarget = (int) (7.8 * angle);

        // turn the 2 motors on:
        robot.spin(0.3);

        // wait until the target position has been reached:
        while (robot.rightMotor.getCurrentPosition() < encoderTarget) {
            idle();
        }

        // turn the 2 motors off:
        robot.stop();

        // communicate with the user:
        // telemetry goes to the driver station phone
        telemetry.addData("Encoder", robot.leftMotor.getCurrentPosition());
        telemetry.update();

        // send messages to the logcat window (Android Monitor):
        System.out.println(TAG + "Encoder (left) = " + robot.leftMotor.getCurrentPosition());
        System.out.println(TAG + "Encoder (right) = " + robot.rightMotor.getCurrentPosition());

        // Send various and flexible messages to the logcat window (Android Monitor):
        Log.i(TAG, "Encoder (left) = " + robot.leftMotor.getCurrentPosition());

        // keep program running for a few more seconds so that
        // we can read the display on the driver station phone:
        sleep(5000);
    }

    void moveRobot( double speed, double inches) {

        // TODO: expand for negative speeds and/or distances
        int encoderTarget = robot.convertInchesToTicks(inches);

        robot.resetEncoders();

        // turn the 2 motors on:
        robot.start(speed);

        // wait until the target position has been reached:
        while (robot.leftMotor.getCurrentPosition() < encoderTarget) {
            idle();   // hand control to Android for a short period of time
        }

        // turn the 2 motors off:
        robot.stop();
    }

    // TODO: write function
    void turnRobot( double speed, double degreed) {

    }
}
