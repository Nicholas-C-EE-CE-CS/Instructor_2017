package edu.elon.robotics;


import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * GamepadP - Line following algorithm using a P-only controller.
 *
 * After initialization and before hitting START on the driver station,
 * the user can change the parameters of the controller.
 * 
 * @author Jochen Fischer
 */
@TeleOp(name="Gamepad P", group="ElonDev")
public class GamepadP extends LinearOpMode {

    // define the robot hardware:
    HardwareDriveBot robot   = new HardwareDriveBot();   // Use the Drivebot hardware

    // other class memebers:
    private ElapsedTime runtime = new ElapsedTime();
    private final String TAG = "MyController";

    // light sensor calibration:
    double reference = -1.0;    // -1 = undefined, will be calibrated further down
    int minBrightness = -1;
    int maxBrightness = -1;

    // PID controller:
    double speed = 0.2;
    double maxTurn = 0.2;   // reasonable upper bound for the turn, used for the I component

    long   dt = 50;             // time interval in milliseconds
    double dT = dt / 1000.0;    // same time interval in seconds
    double Kp = 0.006;

    double error;

    // gamepad related:
    double speedInc = 0.01;     // increment and min value for the speed
    double KpInc    = 0.0005;   // increment and min value for Kp

    @Override
    public void runOpMode() throws InterruptedException {

        // initialize the hardware
        robot.init(hardwareMap);

        // Stay in this loop until the user hits start on the driver station
        // While here, adjust Kp and speed with the gamepad.
        boolean allClearBefore = true;
        while( !isStarted() ) {

            // show usage and modified values on driver station:
            telemetry.addData("B", "calibrate");
            telemetry.addData("d-pad U,D", String.format("Kp = %.4f", Kp));
            telemetry.addData("left bumper,trigger", String.format("speed = %.2f",speed));
            telemetry.addLine()
                    .addData("r", reference)
                    .addData("y", robot.colorSensor.alpha())
                    .addData("min", minBrightness)
                    .addData("max", maxBrightness);
            telemetry.update();

            // use gamepad entry to change values:
            if(gamepad1.b) {
                if(allClearBefore) {
                    // calibrate the light sensor:
                    reference = CalibrateColorSensor();
                    allClearBefore = false;
                }
            }
            else if(gamepad1.dpad_up) {
                if(allClearBefore) {
                    Kp += KpInc;
                    allClearBefore = false;
                }
            }
            else if(gamepad1.dpad_down) {
                if(allClearBefore) {
                    Kp = Math.max(Kp - KpInc, KpInc);
                    allClearBefore = false;
                }
            }
            else if(gamepad1.left_bumper) {
                if(allClearBefore) {
                    speed += speedInc;
                    allClearBefore = false;
                }
            }
            else if(gamepad1.left_trigger > 0.5) {
                if(allClearBefore) {
                    speed = Math.max(speed - speedInc, speedInc);
                    allClearBefore = false;
                }
            }
            else {
                allClearBefore = true;
            }

            if( isStopRequested() ) return;
        }

        // If not alreadys done, calibrate the light sensor:
        if(reference < 0) {
            reference = CalibrateColorSensor();
            telemetry.addData("Reference", reference);
        }

        // print the process parametes in the console:
        Log.i(TAG, String.format("ref = %.1f", reference));
        Log.i(TAG, String.format("dt = %d ms", dt));
        Log.i(TAG, String.format("Kp = %8.6f", Kp));
        Log.i(TAG, String.format("speed = %.2f", speed));

        // Wait for the user to press the PLAY button on the DS:
        //waitForStart();
        runtime.reset();

        // run the PID controller:
        int loopCounter = 0;
        while(opModeIsActive()) {

            // read the current light sensor value:
            int brightness = robot.colorSensor.alpha();

            // implement P controller:
            error = reference - brightness;

            double turn = Kp * error;

            double lPower = speed - turn;
            double rPower = speed + turn;

            robot.leftMotor.setPower(lPower);
            robot.rightMotor.setPower(rPower);

            // update log:
            String message = String.format("%s %d %.1f",
                    runtime.toString(), brightness, error);
            Log.i(TAG, message);

            // update telemetry:
            telemetry.addData(TAG, "running");
            telemetry.addData("dt", String.format("%d ms", dt));
            telemetry.addData("Kp", String.format("%8.6f", Kp));
            telemetry.addData("speed", String.format("%6.4f", speed));
            telemetry.addData("reference", String.format("%.1f [%d,%d]",
                    reference, minBrightness, maxBrightness));
            telemetry.addData("brightness", brightness);
            telemetry.addData("error", error);
            telemetry.update();

            // wait until the next time slot:
            loopCounter++;
            double nextTimeSlot = loopCounter * dt;
            while(runtime.milliseconds() < nextTimeSlot) {
                idle();
            }
        }
    }

    /**
     * CalibrateLightSensor() - Get the reference value needed to run a PID controller.
     *
     * The robot starts roughly on the edge of the line, pointing in driving direction.
     * After hitting INIT on the driver station, the robot moves to the left onto the
     * white line, then to the right off the white line.
     * In this process the maximum and minimum light values are recorded.
     * The mean of the max and min is the reference valu used by the PID controller.
     *
     * @author Jochen Fischer
     * @version 1.0 - 10/9/2016
     *
     * @return reference value for PID controller
     * @throws InterruptedException
     */
    public double CalibrateColorSensor() throws InterruptedException {

        // calibrate the light sensor:
        maxBrightness = robot.colorSensor.alpha();
        minBrightness = maxBrightness;
        int encTarget = (int)(0.25 * HardwareDriveBot.ENCODER_ROTATION_40);  // turn wheels 1/4 of a rotation

        // turn to the left onto the white tape:
        robot.resetEncoders();
        robot.spin(0.1);  // turn very slowly CCW
        while (robot.rightMotor.getCurrentPosition() < encTarget)
        {
            int brightness = robot.colorSensor.alpha();
            if (brightness > maxBrightness) maxBrightness = brightness;
            if (brightness < minBrightness) minBrightness = brightness;
            idle();
        }
        robot.stop();

        robot.resetEncoders();
        robot.spin(-0.1);  // turn very slowly CW
        while (robot.leftMotor.getCurrentPosition() < 2 * encTarget)
        {
            int brightness = robot.colorSensor.alpha();
            if (brightness > maxBrightness) maxBrightness = brightness;
            if (brightness < minBrightness) minBrightness = brightness;
            idle();
        }
        robot.stop();

        double reference = (double)(minBrightness+maxBrightness) / 2.0;
        int threshold = (int) Math.floor(reference);

        // move the robot back right onto the edge of the white tape:
        robot.spin(0.02);  // turn very slowly CCW
        while( robot.colorSensor.alpha() < threshold ) {
            idle();
        }
        robot.stop();

        return reference;
    }

}
