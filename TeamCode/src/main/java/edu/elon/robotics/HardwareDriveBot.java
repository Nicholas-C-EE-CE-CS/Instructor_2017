package edu.elon.robotics;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Hardware class to define, initialize and run the robot.
 *
 * The DriveBot has the following hardware installed:
 *   2 motors
 *   1 touch sensor
 *   1 color sensor
 *   1 ultrasonic sensor
 *
 * @author Jochen Fischer
 * @version 2 - 2017-09-25 as shown in class with some additional comments
 * @version 3 - 2017-09-27 added functions resetEncoders() and convertInchesToTicks()
 * @version 4 - 2017-10-02 added light and touch sensor
 * @version 5 - 2017-10-04 added ultrasonic sensor
 */

public class HardwareDriveBot {

    //----------------------------------------------------------
    // public member variables representing parts of the robot
    //----------------------------------------------------------

    //~~~~~~~~~ motors ~~~~~~~~~~~~~~
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;

    // hardware specific constants:
    public static final double ENCODER_ROTATION_40 = 1120; // encoder ticks per rotation, 40:1 motor
    public static final double ENCODER_ROTATION_60 = 1680; // encoder ticks per rotation, 60:1 motor
    public static final double WHEEL_DIAMETER = 4.0;       // inches
    public static final double TRACK = 10.52;              // inches
    public static final double WHEEL_BASE = 8.5;           // inches

    // other useful constants:
    public static final double STOP = 0.0;
    public static final double SLOW_POWER = 0.2;
    public static final double FULL_POWER = 1.0;

    //~~~~~~~~~ sensors ~~~~~~~~~~~~~~
    public TouchSensor touchSensor = null;
    public ColorSensor colorSensor = null;
    public UltrasonicSensor ultraSonic = null;

    //----------------------------------------------------------
    // local member variables
    //----------------------------------------------------------
    private HardwareMap hwMap = null;
    private ElapsedTime period =  new ElapsedTime();

    //----------------------------------------------------------
    // functions that help operate the robot
    //----------------------------------------------------------

    // Constructor does nothing
    public HardwareDriveBot() {}

    /**
     * Initialize the robot.
     *
     * @author Jochen Fischer
     */
    public void init(HardwareMap ahwMap) {

        // save reference to hardware map:
        hwMap = ahwMap;

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // map motors to hardware
        leftMotor  = hwMap.get(DcMotor.class, "leftMotor");
        rightMotor = hwMap.get(DcMotor.class, "rightMotor");

        // one of the motors needs to be reversed since they are mounted in opposite directions:
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        // reset the motors:
        resetEncoders();

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // map sensors to hardware and initialize them:
        touchSensor = hwMap.get(TouchSensor.class, "touchSensor");

        colorSensor = hwMap.get(ColorSensor.class, "colorSensor");
        colorSensor.enableLed(true);

        ultraSonic = hwMap.get(UltrasonicSensor.class, "ultraSonic");
    }

    /**
     * start, stop, spin - convenience function to run and stop the robot
     *
     * @author Jochen Fischer
     */
    public void stop() {
        leftMotor.setPower(STOP);
        rightMotor.setPower(STOP);
    }

    public void start(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    public void spin(double power) {
        leftMotor.setPower(-power);
        rightMotor.setPower(power);
    }

    /**
     * Stops the robot, resets the encoders and enables the encoders.
     *
     * @author Jochen Fischer
     */
    public void resetEncoders() {
        // reset and use encoders:
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Convertion of travel distance to the equivalent number of motor encoder ticks.
     *
     * @param inches - distance the robot will move in inches
     * @return - number of encoder ticks the motor needs to turn
     *
     * @author Jochen Fischer
     */
    public static int convertInchesToTicks(double inches) {
        double wheelRotations = inches / (Math.PI * HardwareDriveBot.WHEEL_DIAMETER);
        int encoderTicks = (int) (wheelRotations *HardwareDriveBot.ENCODER_ROTATION_40);

        return encoderTicks;
    }

    /**
     * convertTicksToInches - convert number of encoder ticks to inches traveled by the robot
     *
     * @author Jochen Fischer
     * @version 1.0 - 10/1/2016
     *
     * @param encoderTicks  number of encoder ticks the robot has driven
     * @return inches       distance in inches
     */
    public static double convertTicksToInches(int encoderTicks) {
        double wheelRotations = (double) encoderTicks / HardwareDriveBotWithIMU.ENCODER_ROTATION_40;
        double inches = wheelRotations * (Math.PI * HardwareDriveBotWithIMU.WHEEL_DIAMETER);

        return inches;
    }

    /**
     * convertDegreesToTicks - convert turn angle to encoder ticks
     *
     * @param degrees  turn angle of the robot, positive values are clockwise
     * @return number of encoder ticks
     *
     * @author Jochen Fischer
     */
    public static int convertDegreesToTicks(double degrees) {
        // distance the wheels need to travel divided by the wheel circumference:
        double wheelRotations = (degrees / 360.0) * Math.PI * HardwareDriveBot.TRACK
                / (Math.PI * HardwareDriveBot.WHEEL_DIAMETER);
        int encoderTarget = (int)(wheelRotations * HardwareDriveBot.ENCODER_ROTATION_40);

        return encoderTarget;
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
