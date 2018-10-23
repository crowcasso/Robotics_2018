package edu.elon.cs.robotics;

/**
 * Autonomously drive the robot.
 *
 * @author J. Hollingsworth
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Autonomous Test", group="DriveBot")
public class AutonomousTest extends LinearOpMode {

    private RobotHardware robot = null;

    public void runOpMode() throws InterruptedException {

        // define the robot
        robot = new RobotHardware(hardwareMap);

        waitForStart();

        drive(24, 0.3);
        turn(90, 0.2);

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
