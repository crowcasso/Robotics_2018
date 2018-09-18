package org.firstinspires.ftc.teamcode;

/**
 * Autonomously drive the robot.
 *
 * @author J. Hollingsworth
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Autonomous Test", group="DriveBot")
public class AutonomousTest extends LinearOpMode {

    RobotHardware robot = null;

    public void runOpMode() throws InterruptedException {

        // define the robot
        robot = new RobotHardware(hardwareMap);

        waitForStart();

        // reset the encoder values to 0
        robot.resetEncoders();

        // drive for 12 inches
        int distanceInTicks = robot.convertInchesToTicks(12.0);
        robot.start(RobotHardware.SLOW_POWER);
        while (robot.leftMotor.getCurrentPosition() < distanceInTicks && opModeIsActive()) {
            idle();
        }

        // stop the robot
        robot.stop();

    }

}
