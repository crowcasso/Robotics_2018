package edu.elon.cs.robotics;

/**
 * Autonomously drive and park the robot.
 *
 * @author J. Hollingsworth
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Autonomous Parking", group="DriveBot")
public class AutonomousParking extends LinearOpMode {

    private RobotHardware robot = null;

    public void runOpMode() throws InterruptedException {

        // define the robot
        robot = new RobotHardware(hardwareMap);

        waitForStart();

        driveUntilWhite(findThreshold(12));
        turn(90, robot.SLOW_POWER);
        int parkingSpot = findBestParkingSpot();

        while (opModeIsActive());
    }

    private int findBestParkingSpot() {

        double closeDistance = 999.99;
        double farDistance = -999.99;
        boolean isWall = true;
        int startOpeningTick = 0, endOpeningTick = 0;
        double maxWidth = -999.99;
        int bestParking = 0;

        robot.resetEncoders();
        robot.start(robot.NORMAL_POWER);
        while (robot.touchSensor.getState() == true && opModeIsActive()) {

            double distance = 1.0; //robot.distanceSensor.getDistance(DistanceUnit.CM);
            if (distance > farDistance) farDistance = distance;
            if (distance < closeDistance) closeDistance = distance;

            telemetry.addData("Current Distance: ", distance);
            telemetry.addData("Max Distance: ", farDistance);
            telemetry.addData("Min Distance: ", closeDistance);

            if (isWall && Math.abs(distance - farDistance) < 5) {
                startOpeningTick = robot.leftMotor.getCurrentPosition();
                isWall = false;
            }

            if (!isWall && Math.abs(distance - closeDistance) < 5) {
                endOpeningTick = robot.leftMotor.getCurrentPosition();

                double width = robot.convertTicksToInches(endOpeningTick - startOpeningTick);
                if (width > robot.WHEEL_BASE) {
                    telemetry.addData("Parking Spot (width): ", width);
                }

                if (width > maxWidth) {
                    maxWidth = width;
                    bestParking = endOpeningTick;
                    telemetry.addData("Best Parking Spot: ", bestParking);
                }

                isWall = true;
            }

            telemetry.update();
            idle();
        }

        return bestParking;
    }

    private void driveUntilWhite(int threshold) {

        // move the robot
        robot.resetEncoders();
        robot.start(robot.NORMAL_POWER);
        while (robot.colorSensor.alpha() > threshold && opModeIsActive()) {
            idle();
        }
        robot.stop();
    }

    private int findThreshold(double distance) {

        // find a positive distance to travel
        double distanceMagnitude = Math.abs(distance);
        int distanceInTicks = robot.convertInchesToTicks(distanceMagnitude);

        int minBrightness = 999;
        int maxBrightness = -999;

        // move the robot
        robot.resetEncoders();
        robot.start(robot.NORMAL_POWER);
        while (Math.abs(robot.leftMotor.getCurrentPosition()) < distanceInTicks && opModeIsActive()) {

            // search for max/min color
            int brightness = robot.colorSensor.alpha();
            if (brightness > maxBrightness) maxBrightness = brightness;
            if (brightness < minBrightness) minBrightness = brightness;

            idle();
        }
        robot.stop();

        return ((maxBrightness - minBrightness) / 2) + minBrightness;
    }

    private void drive(double distance, double speed) {

        // make sure the speed value make sense
        speed = Range.clip(speed, -1.0, 1.0);

        // find a positive distance to travel
        double distanceMagnitude = Math.abs(distance);
        int distanceInTicks = robot.convertInchesToTicks(distanceMagnitude);

        // handle all 4 combinations of +/- speed,distance
        speed = speed * Math.signum(distance);

        // move the robot
        robot.resetEncoders();
        robot.start(speed);
        while (Math.abs(robot.leftMotor.getCurrentPosition()) < distanceInTicks && opModeIsActive()) {
            idle();
        }
        robot.stop();
    }

    private void turn(double degrees, double speed) {

        // make sure the speed value make sense
        speed = Range.clip(speed, -1.0, 1.0);

        // handle all 4 combinations of +/- speed,degrees
        speed = speed * Math.signum(degrees);

        int degreesInTicks = robot.convertDegreesToTicks(Math.abs(degrees));

        robot.resetEncoders();
        robot.spin(speed);
        while (Math.abs(robot.leftMotor.getCurrentPosition()) < degreesInTicks && opModeIsActive()) {
            idle();
        }
        robot.stop();
    }

}
