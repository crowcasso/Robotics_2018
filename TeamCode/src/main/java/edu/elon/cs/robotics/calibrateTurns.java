package edu.elon.cs.robotics;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;


@Autonomous(name="calibrateTurns", group="DriveBot")
public class calibrateTurns extends LinearOpMode {

    DriveBot robot = null;

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize  the hardware
        robot = new DriveBot(hardwareMap);
        // wait for the user to press the START button
        waitForStart();

        double finalHeading = 0.0;
        Log.i("TurnCalibrate", "Calibrating Turn");
        robot.zeroGyro();
        turnByAngle(90,  0.3);
        finalHeading = robot.getRelativeHeading();
        Log.i("TurnCalibrate", "turn 90   " + String.valueOf(finalHeading));
        telemetry.addLine().addData("turn 90", finalHeading);
        telemetry.update();
        waitForTouch();


        robot.zeroGyro();
        turnByAngle(60,  0.3);
        finalHeading = robot.getRelativeHeading();
        Log.i("TurnCalibrate", "turn 60   " + String.valueOf(finalHeading));
        telemetry.addLine().addData("turn 60", finalHeading);
        telemetry.update();
        waitForTouch();

        robot.zeroGyro();
        turnByAngle(45,  0.3);
        finalHeading = robot.getRelativeHeading();
        Log.i("TurnCalibrate", "turn 45   " + String.valueOf(finalHeading));
        telemetry.addLine().addData("turn 45", finalHeading);
        telemetry.update();
        waitForTouch();

        robot.zeroGyro();
        turnByAngle(-170,  0.3);
        finalHeading = robot.getRelativeHeading();
        Log.i("TurnCalibrate", "turn -170   " + String.valueOf(finalHeading));
        telemetry.addLine().addData("turn -170", finalHeading);
        telemetry.update();
        waitForTouch();

        robot.zeroGyro();
        turnByAngle(170,  0.3);
        finalHeading = robot.getRelativeHeading();
        Log.i("TurnCalibrate", "turn 170   " + String.valueOf(finalHeading));
        telemetry.addLine().addData("turn 170", finalHeading);
        telemetry.update();
        waitForTouch();

        robot.zeroGyro();
        turnByAngle(-30,  0.3);
        finalHeading = robot.getRelativeHeading();
        Log.i("TurnCalibrate", "turn -30   " + String.valueOf(finalHeading));
        telemetry.addLine().addData("turn -30", finalHeading);
        telemetry.update();
        waitForTouch();
    }


    public boolean waitForTouch() {
        while (!robot.touchSensor.isPressed() && opModeIsActive()) {
            idle();
        }
        return true;
    }
    // drive by distance
    public void turnByAngle(double turnAngle, double power){

        if (power == 0 || turnAngle == 0) {
            return;
        }

        boolean clockwise = (turnAngle < 0);

        turnAngle = Math.abs(turnAngle);
        power = Range.clip(Math.abs(power), 0, 1.0);

        int targetTicks = robot.convertAngleToTicks(turnAngle);
        robot.resetMotors();

        if(clockwise){
            robot.turnInPlace(power);
        }
        else {
            robot.turnInPlace(-power);
        }

        while(Math.abs(robot.rightMotor.getCurrentPosition()) < targetTicks && opModeIsActive()) {
            telemetry.addData("target", targetTicks);
            telemetry.addData("value", robot.rightMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }

        // turn the drive motor off
        robot.stop();
    }

    private int correctCount = 0;
    /*
     * @param gyroTarget The target heading in degrees, between 0 and 360
     * @param gyroRange The acceptable range off target in degrees, usually 1 or 2
     * @param gyroActual The current heading in degrees, between 0 and 360
     * @param minSpeed The minimum power to apply in order to turn (e.g. 0.05 when moving or 0.15 when stopped)
     * @param addSpeed The maximum additional speed to apply in order to turn (proportional component), e.g. 0.3
     * @return The number of times in a row the heading has been in the range
     */
    public int gyroCorrect(double gyroTarget, double gyroRange, double gyroActual, double minSpeed, double addSpeed) {
        double delta = (gyroTarget - gyroActual + 360.0) % 360.0; //the difference between target and actual mod 360
        if (delta > 180.0) delta -= 360.0; //makes delta between -180 and 180
        if (Math.abs(delta) > gyroRange) { //checks if delta is out of range
            this.correctCount = 0;
            double gyroMod = delta / 45.0; //scale from -1 to 1 if delta is less than 45 degrees
            if (Math.abs(gyroMod) > 1.0) {
                gyroMod = Math.signum(gyroMod); //set gyromod to 1 or -1 if the error is more than 45 degrees
            }
            robot.turnInPlace(minSpeed * Math.signum(gyroMod) + addSpeed * gyroMod);
        }
        else {
            this.correctCount++;
            robot.turnInPlace(0.0);
        }
        return this.correctCount;
    }
}
