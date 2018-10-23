package edu.elon.cs.robotics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Drive with Joystick", group="DriveBot")
public class DriveJoystick extends LinearOpMode {

    private RobotHardware robot = null;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotHardware(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            runtime.reset();

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            telemetry.addLine()
                    .addData("drive", "%.3f", drive)
                    .addData("turn", "%.3f", turn);

            leftPower = drive + turn;   // needs to be limited to [-1,1]
            rightPower = drive - turn;  // needs to be limited to [-1,1]

            leftPower    = Range.clip(leftPower,  -1.0, 1.0) ;
            rightPower   = Range.clip(rightPower, -1.0, 1.0) ;

            // Send calculated power to wheels
            robot.rightMotor.setPower(rightPower);
            robot.leftMotor.setPower(leftPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();

            idle();
        }

    }
}
