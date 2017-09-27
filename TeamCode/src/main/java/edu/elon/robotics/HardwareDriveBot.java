package edu.elon.robotics;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Hardware class to define, initialize and run the robot.
 *
 * The DriveBot has the following hardware installed:
 *   - motorLeft
 *   - motorRight
 *
 * @author Jochen Fischer
 * @version 2 - 2017-09-25 as shown in class with some additional comments
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
    // NORMAL_POWER

    //----------------------------------------------------------
    // local member variables
    //----------------------------------------------------------
    private HardwareMap hwMap = null;

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

        // map motors to hardware
        leftMotor  = hwMap.get(DcMotor.class, "leftMotor");
        rightMotor = hwMap.get(DcMotor.class, "rightMotor");

        // reset the motors:
        resetEncoders();
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

    // TODO: add comments...
    public void resetEncoders() {
        // one of the motors needs to be reversed since they are mounted in opposite directions:
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        // reset and use encoders:
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int convertInchesToTicks(double inches) {
        double rotations = inches / (Math.PI * HardwareDriveBot.WHEEL_DIAMETER);
        int encoderTicks = (int) (rotations *HardwareDriveBot.ENCODER_ROTATION_40);

        return encoderTicks;
    }

    // TODO: Assignment 1 - convertDegreesToTicks

}
