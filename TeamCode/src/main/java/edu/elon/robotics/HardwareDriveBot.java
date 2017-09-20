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
 */

public class HardwareDriveBot {
    //----------------------------------------------------------
    // public member variables representing parts of the robot
    //----------------------------------------------------------

    //~~~~~~~~~ motors ~~~~~~~~~~~~~~
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;

    //----------------------------------------------------------
    // local member variables
    //----------------------------------------------------------
    private HardwareMap hwMap = null;

    //----------------------------------------------------------
    // functions that help opereate the robot
    //----------------------------------------------------------

    // Constructor does nothing
    public HardwareDriveBot() {

    }

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

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
    }
}
