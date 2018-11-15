package edu.elon.cs.robotics;

/**
 * A simple Robot Hardware class.
 *
 * @author J. Hollingsworth
 */

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class RobotHardware {

    // some common speeds
    public static final double STOP_POWER = 0.0;
    public static final double SLOW_POWER = 0.2;
    public static final double NORMAL_POWER = 0.6;
    public static final double FULL_POWER = 1.0;

    // motors and wheels
    public static final double TICKS_PER_ROTATION = 1120;
    public static final double WHEEL_DIAMETER = 4.0;
    public static final double TRACK = 9.75;
    public static final double WHEEL_BASE = 10.75;
    public static final double INCHES_PER_ROTATION = 13.125;
    public static final double TICKS_PER_INCH = TICKS_PER_ROTATION / INCHES_PER_ROTATION;

    // motors
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;

    // sensors
    public DigitalChannel touchSensor = null;
    public ColorSensor colorSensor = null;
    //public DistanceSensor distanceSensor = null;

    public RobotHardware(HardwareMap hardwareMap) {

        // map the motors to hardware
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        // reverse one of the motors
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        resetEncoders();

        // setup the sensors
        touchSensor = hardwareMap.get(DigitalChannel.class, "touchSensor");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        colorSensor.enableLed(true);
        //distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
    }

    public void stop() {
        leftMotor.setPower(STOP_POWER);
        rightMotor.setPower(STOP_POWER);
    }

    public void spin(double power) {
        leftMotor.setPower(-power);
        rightMotor.setPower(power);
    }

    public void start(double power) {
        power = Range.clip(power, -1.0, 1.0);
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    public void resetEncoders() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int convertInchesToTicks(double inches) {
        double wheelRotations = inches / (Math.PI * WHEEL_DIAMETER);
        return (int) (wheelRotations * TICKS_PER_ROTATION);
    }

    public double convertTicksToInches(int ticks) {
        return (Math.PI * WHEEL_DIAMETER * ticks) / TICKS_PER_ROTATION;
    }

    public int convertDegreesToTicks(double degrees) {
        double wheelRotations = (degrees / 360.0) * Math.PI * TRACK / (Math.PI * WHEEL_DIAMETER);
        return (int) (wheelRotations * TICKS_PER_ROTATION);
    }

}
