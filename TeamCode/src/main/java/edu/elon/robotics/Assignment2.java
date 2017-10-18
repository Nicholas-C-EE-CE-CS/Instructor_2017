package edu.elon.robotics;


import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Assignment 2 - Counting lines.
 *
 * @author Jochen Fischer
 * @version 1.0, 9/18/2016
 *
 * The program has three parts:
 *
 * 1. Calibrate the light sensor, but moving slowly over a white line.
 * 2. Drive the robot to the opposing wall and count the white lines on the way.
 * 3. Move the robot back to base.
 *
 * Note: the instructor has left some comments in the code to illustrate the
 * thought process during development.
 */
@Autonomous(name="Assignment2", group="Elon")
public class Assignment2 extends LinearOpMode {

    // define the robot hardware:
    HardwareDriveBot robot   = new HardwareDriveBot();   // Use the Drivebot hardware
    String TAG = "CountingLines";

    @Override
    public void runOpMode() throws InterruptedException {

        // initialize the robot:
        // The robot now has touch and light sensor mounted.
        // This will also include resetting the motor encoders.
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the user to press the PLAY button on the DS:
        waitForStart();

        // move the robot by 12 inches and record the minumum and maximum light sensor values:
        int encTarget = robot.convertInchesToTicks(12.0);
        int minA = robot.colorSensor.alpha();
        int maxA = minA;

        // drive the robot slowly over the white line:
        robot.start(0.15);
        while( robot.leftMotor.getCurrentPosition() < encTarget ) {
            int A = robot.colorSensor.alpha();
            if (A > maxA) maxA = A;
            if (A < minA) minA = A;
            idle();
        }
        // stop the robot during devolopment
        robot.stop();

        // pick two thresholds at 60% (up) and 40% (down):
        int upTreshold = minA + (int) Math.round(0.6 * (maxA-minA));
        int downTreshold = minA + (int) Math.round(0.4 * (maxA-minA));

        Log.i(TAG, String.format("min=%d  max=%d  up=%d  down=%d",
                minA, maxA, upTreshold, downTreshold));

        // give user a chance to read the display during development:
        telemetry.addData("min Alpha", minA);
        telemetry.addData("max Alpha", maxA);
        telemetry.addData("upThreshold", upTreshold);
        telemetry.addData("downThreshold", downTreshold);
        telemetry.update();
        sleep(4000);

        // alternatively, the waitForStart() function could be moved down here.

        // set up the line counting variables:
        boolean wasWhite = false;
        int lineCount = 0;
        double inches;
        telemetry.addData("lines", lineCount);
        telemetry.update();

        // keep going until we hit the wall:
        robot.start(HardwareDriveBot.SLOW_POWER);

        while( !robot.touchSensor.isPressed() ) {
            // count the lines:
            int A = robot.colorSensor.alpha();

            if (!wasWhite && A > upTreshold  ) {
                // a transition from black to white was detected:
                lineCount++;
                wasWhite = true;
            }
            else if (wasWhite && A < downTreshold  ) {
                // a transition from white to black was detected:
                wasWhite = false;
            }

            // get the distance traveled:
            inches = robot.convertTicksToInches(robot.leftMotor.getCurrentPosition());
            telemetry.addData("inches", String.format("%.1f", inches));
            telemetry.addData("lines", lineCount);
            telemetry.update();

            // example for continuous logging to console
            // (needs waitForTick() to limit the amount of data)
            Log.i(TAG, String.format("A=%3d  wasWite=%s  inches=%5.1f  lines=%2d",
                    A, wasWhite?"YES":"NO ", inches, lineCount));

            // Pause for metronome tick. 50ms each cycle or 20 updates per second.
            robot.waitForTick(50);
        }
        robot.stop();

        // read the left encoder value and use it to drive back to base:
        inches = robot.convertTicksToInches(robot.leftMotor.getCurrentPosition());
        telemetry.addData("inches", inches);
        telemetry.addData("lines", lineCount);
        telemetry.update();
        Log.i(TAG, String.format("inches=%.1f", inches));
        Log.i(TAG, String.format("lines=%d", lineCount));

        // drive back to base:
        moveRobot(HardwareDriveBot.SLOW_POWER, -inches);

        // keep display until the user hits STOP:
        while (opModeIsActive()) {
            idle();
        }
    }

    /**
     * @author Jochen Fischer
     * @version 1.0 - 9/25/2016 - inital version, works only going forward
     * @version 1.1 - 9/26/2016 - added code to go in both directions
     * @version 2.0 - 10/4/2016 - extensive use of functions
     *
     * moveRobot - Moves the robot by a given distance and speed.
     *
     * @param speed    robot speed between -1.0 ... 1.0
     * @param inches   driving distance in inces
     * @throws InterruptedException
     */
    private void moveRobot(double speed, double inches) throws InterruptedException {

        // determine if the robot moves forward or backward:
        double direction = Math.signum(speed * inches);

        // sanity check: don't do anything if either speed or inches is zero
        if(direction == 0.0) return;

        // since we know in which direction the robot moves,
        // we can use absolute (=positive) values for both the encoder value and target

        // translate the distance in inches to encoder ticks:
        int encoderTarget = robot.convertInchesToTicks( Math.abs(inches) );

        // move the desired distance:
        robot.resetEncoders();
        robot.start(Math.abs(speed) * direction);
        while (Math.abs(robot.leftMotor.getCurrentPosition()) < encoderTarget) {
            idle();
        }
        robot.stop();
    }
}
