package edu.elon.cs.robotics;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="FindKCritical", group="Current")
public class FindKCritical extends LinearOpMode {
/**
 * Line follower with proportional controller (P-Controller)
 *
 * @author Kyle Altmann
 */

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    String TAG = "KNALine";

    // declare robot class:
    DriveBot robot;

    int hThresh = 20;
    int lThresh = 20;
    int midLight = 26;

    double speed   = 0.3;
    double maxTurn = Math.max(Math.min((speed-0.1), (1.0-speed)), 0.05);

    int reference = 26;
    long   dt = 50;            // time interval in milliseconds
    double dT = dt / 1000.0;   // same time interval in seconds
    double Kp = 0.001;         // proportional gain


    double error;
    double turn;

    int loopCounter = 0;
    double startNextLoop = 0.0;


    @Override
    public void runOpMode() {

        robot = new DriveBot(hardwareMap);
        // wait for the user to press the START button


        waitForStart();

        calibrateLightSwivel();
        reference = midLight;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // run until the driver presses STOP
        while (opModeIsActive()) {
            telemetry.addData("Running with Kp = ", Kp);
            telemetry.addData("Next Step: ", "Touch the Sensor to stop run");
            telemetry.update();
            runtime.reset();
            loopCounter = 0;
            startNextLoop = 0.0;
            while (!robot.touchSensor.isPressed() && opModeIsActive()){
                // read light sensor:
                int brightness = robot.colorSensor.alpha();

                // implement PID-controller:
                error = reference - brightness;

                // positive "turn" will turn the robot left (CCW):
                turn = Kp * error;

                // set motors
                double leftPower = speed - turn;
                double rightPower = speed + turn;

                // Send calculated power to wheels
                robot.leftMotor.setPower(leftPower);
                robot.rightMotor.setPower(rightPower);

                // Capture parameters in logcat:
                String message = String.format("%s %2d %6.4f %6.4f %6.4f",
                        runtime.toString(), brightness, Kp, speed, turn);
                Log.i(TAG, message);

                // run loop in increments of dt:
                loopCounter++;
                startNextLoop = loopCounter * dt;
                while(runtime.milliseconds() < startNextLoop) {
                    idle();
                }
            }
            robot.stop();
            sleep(1000);
            telemetry.addData("Next Step: ", "Return Robot and Touch the Sensor");
            telemetry.update();
            while (!robot.touchSensor.isPressed() && opModeIsActive()){
                idle();
            }
            sleep(2000);
            Kp = Kp + .001;
            String message = String.format("*********NEXT RUN********* %6.4f %6.4f", Kp, speed);
            Log.i(TAG, message);
        }
    }

    public void calibrateLightSwivel(){
        int reading=3;
        int whiteLight = 0;
        int blackLight = 300;

        robot.resetMotors();
        robot.zeroGyro();
        robot.turnInPlace(-0.2);

        while(robot.getRelativeHeading() < 45 && opModeIsActive()) {
            reading = robot.colorSensor.alpha();
            if(reading > whiteLight){
                whiteLight = reading;
            }
            if(reading < blackLight){
                blackLight = reading;
            }
            //telemetry.addData("high", whiteLight);
            //telemetry.addData("low", blackLight);
            //telemetry.addData("current", reading);
            //telemetry.update();
            idle();
        }
        robot.stop();

        robot.turnInPlace(0.2);
        while(robot.getRelativeHeading() > -45 && opModeIsActive()) {
            reading = robot.colorSensor.alpha();
            if(reading > whiteLight){
                whiteLight = reading;
            }
            if(reading < blackLight){
                blackLight = reading;
            }
            //telemetry.addData("high", whiteLight);
            //telemetry.addData("low", blackLight);
            //telemetry.addData("current", reading);
            //telemetry.update();
            idle();
        }
        robot.stop();
        hThresh = blackLight + (int)((whiteLight-blackLight)*2./5.);
        lThresh = blackLight + (int)((whiteLight-blackLight)*1./5.);
        midLight = (int)((whiteLight-blackLight)/2.0);

        //position back near edge
        robot.turnInPlace(-0.1);
        while(robot.colorSensor.alpha() < midLight && opModeIsActive()) {
            //telemetry.addData("reference", midLight);
            //telemetry.addData("current", reading);
            //telemetry.update();
            idle();
        }
        robot.stop();
    }
}