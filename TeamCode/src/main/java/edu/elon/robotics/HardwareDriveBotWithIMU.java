package edu.elon.robotics;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.Locale;

/**
 * Hardware class to define, initialize and run the robot.
 *
 * @author Jochen Fischer
 *
 */

public class HardwareDriveBotWithIMU {

    //------------------------------------------------------------------
    // public members variables representing the parts on the robot:
    //------------------------------------------------------------------

    //~~~~~~~~~~ motors ~~~~~~~~~~
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;

    // hardware specific constants:
    public static final double ENCODER_ROTATION_40 = 1120;
    public static final double WHEEL_DIAMETER = 4.0;  // inches
    public static final double TRACK = 10.6;          // inches
    public static final double WHEEL_BASE = 8.5;      // inches

    // useful constants:
    public static final double STOP = 0.0;
    public static final double SLOW_POWER = 0.20;
    public static final double NORMAL_POWER = 0.50;
    public static final double FULL_POWER = 1.00;

    //~~~~~~~~~~ sensors ~~~~~~~~~~
    // will be added later here...
    public ColorSensor colorSensor = null;
    public TouchSensor touchSensor = null;
    public UltrasonicSensor ultraSonic = null;

    // IMU:
    public BNO055IMU imu;
    public boolean calibratedIMU;

    //------------------------------------------------------------------
    // local member variables:
    //------------------------------------------------------------------
    private HardwareMap hwMap  =  null;
    private ElapsedTime period =  new ElapsedTime();

    //------------------------------------------------------------------
    // Functions that help to operate the robot
    //------------------------------------------------------------------

    /* Constructor does nothing... */
    public HardwareDriveBotWithIMU() {

    }

    /**
     * Initialize the robot hardware:
     *
     * @author Jochen Fischer
     */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map:
        hwMap = ahwMap;

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Map and initialize the motors:
        leftMotor  = hwMap.get(DcMotor.class, "leftMotor");
        rightMotor = hwMap.get(DcMotor.class, "rightMotor");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        // robot uses encoders on the drive motors:
        resetEncoders();

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Map and initialize the sensors:
        colorSensor = hwMap.get(ColorSensor.class, "colorSensor");
        colorSensor.enableLed(true);

        touchSensor = hwMap.get(TouchSensor.class, "touchSensor");
        ultraSonic = hwMap.get(UltrasonicSensor.class, "ultraSonic");

        //------------------------------------------------------------
        // IMU - BNO055
        // Set up the parameters with which we will use our IMU.
        // + 9 degrees of freedom
        // + use of calibration file (see calibration program)
        //------------------------------------------------------------
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitImuCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.mode                = BNO055IMU.SensorMode.NDOF;

        parameters.accelerationIntegrationAlgorithm = null;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Read the IMU configuration from the data file saved during calibration.
        // Using try/catch allows us to be specific about the error instead of
        // just showing a NullPointer exception that could come from anywhere in the program.
        calibratedIMU = true;
        try {
            File file = AppUtil.getInstance().getSettingsFile(parameters.calibrationDataFile);
            String strCalibrationData = ReadWriteFile.readFile(file);
            BNO055IMU.CalibrationData calibrationData = BNO055IMU.CalibrationData.deserialize(strCalibrationData);
            imu.writeCalibrationData(calibrationData);
        }
        catch (Exception e) {
            calibratedIMU = false;
        }

    }


    /**
     * resetEncoders - stops the robot and resets the encoders for the left and right motor
     *
     * @author Jochen Fischer
     * @version 1.0 - 9/16/2017
     */
    public void resetEncoders() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    /**
     * start, spin, stop - convenience functions to run and stop the robot:
     *
     * @author Jochen Fischer
     * @version 1.0 - 9/13/2017
     */
    public void start(double speed) {
        leftMotor.setPower(speed);
        rightMotor.setPower(speed);
    }

    public void spin(double speed) {    // positive turn is CCW when viewed from top
        leftMotor.setPower(-speed);     // left motor turns backward for CCW turn
        rightMotor.setPower(speed);     // right motor turns forward for CCW turn
    }
    public void stop() {
        leftMotor.setPower(STOP);
        rightMotor.setPower(STOP);
    }

    /**
     * convertInchesToTicks - convert a distance given in inches to motor encoder ticks
     *
     * @author Jochen Fischer
     * @version 1.0 - 10/1/2016
     *
     * @param inches  distance in inches
     * @return ticks  encoder ticks
     */
    public static int convertInchesToTicks(double inches) {

        // translate the distance in inches to encoder ticks:
        double wheelRotations = inches / (Math.PI * HardwareDriveBotWithIMU.WHEEL_DIAMETER);
        int encoderTicks = (int)(wheelRotations * HardwareDriveBotWithIMU.ENCODER_ROTATION_40);

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
     * @author Jochen Fischer
     * @version 1.0 - 10/5/2016
     *
     * @param degrees  turn angle of the robot, positive values are clockwise
     * @return
     */
    public static int convertDegreesToTicks(double degrees) {
        double wheelRotations = (degrees / 360.0) * Math.PI * HardwareDriveBotWithIMU.TRACK
                / (Math.PI * HardwareDriveBotWithIMU.WHEEL_DIAMETER);
        int encoderTarget = (int)(wheelRotations * HardwareDriveBotWithIMU.ENCODER_ROTATION_40);

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

    /***
     * Formatting functions for angles and degrees.
     */
    public String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    public String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
