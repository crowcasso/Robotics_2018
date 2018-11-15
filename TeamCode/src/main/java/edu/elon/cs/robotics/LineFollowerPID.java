package edu.elon.cs.robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Line Follower PID", group="PID")
public class LineFollowerPID extends LinearOpMode {

    private final String TAG = "MY_ROBOT";

    private RobotHardware robot = null;
    private ElapsedTime runTime = null;

    // basic setup:
    double speed = 0.2;
    double maxTurn = 0.2;

    double reference = -1.0;
    int minBrightness = -1;
    int maxBrightness = -1;

    // PID controller parameters:
    double Kc = 0.00015; //0.0002;
    double Pc = 0.8;
    long dt = 50;
    double dT = dt / 1000.0;

    // dependent PID controller values:
    double Kp = 0.6 * Kc;
    double Ki = 0.0; //2.0 * Kp * dT / Pc;
    double Kd = 0.0; //Kp * Pc / (8.0 * dT);

    double error;
    double previousError;
    double dError;
    double sumError;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotHardware(hardwareMap);
        runTime = new ElapsedTime();

        reference = calibrateColorSensor();

        waitForStart();

        runTime.reset();
        int loopCounter = 0;
        while (opModeIsActive()) {

            // read the current light sensor value:
            int brightness = robot.colorSensor.alpha();

            // implement PID
            error = reference - brightness;
            sumError = 0.9 * sumError + error;
            sumError = Range.clip(sumError, -maxTurn/Ki, maxTurn/Ki);
            dError = error - previousError;
            previousError = error;

            double turn = Kp * error + Ki * sumError + Kd * dError;

            double leftPower = speed - turn;
            double rightPower = speed + turn;

            robot.leftMotor.setPower(leftPower);
            robot.rightMotor.setPower(rightPower);

            // update telemetry:
            telemetry.addData(TAG, "running");
            telemetry.addData("dt ", String.format("%d ms", dt));
            telemetry.addData("Pc ", String.format("%.2f sec", Pc));
            telemetry.addData("Kc ", String.format("%8.6f", Kc));
            telemetry.addData("Kp ", String.format("%8.6f", Kp));
            telemetry.addData("Ki ", String.format("%8.6f", Ki));
            telemetry.addData("Kd ", String.format("%8.6f", Kd));
            telemetry.addData("speed", String.format("%6.4f", speed));
            telemetry.addData("reference", String.format("%.1f [%d,%d]",
                    reference, minBrightness, maxBrightness));
            telemetry.addData("brightness", brightness);
            telemetry.addData("error", error);
            telemetry.update();


            // wait until the next time slot
            loopCounter++;
            double nextTimeSlot = loopCounter * dt;
            while (runTime.milliseconds() < nextTimeSlot) {
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
    public double calibrateColorSensor() throws InterruptedException {

        // calibrate the light sensor:
        maxBrightness = robot.colorSensor.alpha();
        minBrightness = maxBrightness;
        int encTarget = (int)(0.25 * RobotHardware.TICKS_PER_ROTATION);  // turn wheels 1/4 of a rotation

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
        robot.spin(0.05);  // turn very slowly CCW
        while( robot.colorSensor.alpha() < threshold ) {
            idle();
        }
        robot.stop();

        return reference;
    }

}
