package edu.elon.cs.robotics;

//**************************
import com.qualcomm.hardware.bosch.BNO055IMU;
//**************************

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.Locale;

import com.qualcomm.robotcore.util.Range;

// defines the simple DriveBot robot.

public class DriveBot {

    // useful constants
    public static final double STOP_POWER = 0.0;
    public static final double SLOW_POWER = 0.2;
    public static final double FULL_POWER = 1.0;
    public static final double WHEEL_DIAMETER = 10.33;
    public static final double TRACK = 28.2425;
    public static final int TICKS_PER_ROTATION = 1680;
    public static final double CM_PER_INCH = 2.54;

    // motors
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;

    // sensors
    public ColorSensor colorSensor = null;
    public TouchSensor touchSensor = null;
    public UltrasonicSensor ultraSonic = null;

//**************************
    // IMU:
    public BNO055IMU imu;
    public double headingResetValue = 0.0;
//**************************


    // Constructor that initializes the robot.

    public DriveBot(HardwareMap hardwareMap) {
        // map motors to hardware
        leftMotor = hardwareMap.get(DcMotor.class,"motorL");
        rightMotor = hardwareMap.get(DcMotor.class, "motorR");

        // one motor needs to be reversed since they are mounted in opposite directions
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Map and initialize the sensors:
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        colorSensor.enableLed(true);

        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");
        ultraSonic = hardwareMap.get(UltrasonicSensor.class, "ultraSonic");

	//**************************
        //------------------------------------------------------------
        // IMU - BNO055
        // Set up the parameters with which we will use our IMU.
        // + 9 degrees of freedom
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
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        resetMotors();
        zeroGyro();
    //**************************
    }

    // start, stop convenience methods to run/stop the robot

    public void resetMotors() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void stop() {
        leftMotor.setPower(STOP_POWER);
        rightMotor.setPower(STOP_POWER);
    }

    public void turnInPlace(double power) {
        power = Range.clip(power, -1.0, 1.0); // positive turn is CCW when viewed from top
        leftMotor.setPower(power);
        rightMotor.setPower(-power);
    }

    public void start(double power) {
      power = Range.clip(power, -1.0, 1.0);
      leftMotor.setPower(power);
      rightMotor.setPower(power);
    }

//**************************
    // Turning with Gyro

    public double zeroGyro(){
        headingResetValue = getAbsoluteHeading();
        return headingResetValue;
    }

    public double getAbsoluteHeading(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public double getRelativeHeading(){
        double head = getAbsoluteHeading() - headingResetValue;
        head = (head + 360) % 360;
        if(head > 180){
            head = head - 360;
        }
        return head;
    }
//**************************

    // distance conversions

    public double convertTicksToCM(double numTicks){
        double circleFrac = numTicks/DriveBot.TICKS_PER_ROTATION;
        double wheelCircumference = Math.PI * DriveBot.WHEEL_DIAMETER;
        double distance = circleFrac*wheelCircumference;
        return distance;
    }

    public int convertCMToTicks(double distInCM){
        double wheelCircumference = Math.PI * DriveBot.WHEEL_DIAMETER;
        double circleFrac = distInCM/wheelCircumference;
        int numTicks = (int)(circleFrac*DriveBot.TICKS_PER_ROTATION);

        return numTicks;
    }

    public int convertInchesToTicks(double distInInches){

        int numTicks = convertCMToTicks(distInInches*CM_PER_INCH);
        return numTicks;
    }

    public int convertAngleToTicks(double turnAngle){

        double turnCircumference = Math.PI * DriveBot.TRACK;
        double wheelDistance = turnAngle/360.*turnCircumference;

        int numTicks = convertCMToTicks(wheelDistance);
        return numTicks;
    }
}
