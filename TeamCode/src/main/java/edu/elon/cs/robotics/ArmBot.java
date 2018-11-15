package edu.elon.cs.robotics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class ArmBot extends LinearOpMode {

    private HardwareArmBot armBot = null;



    @Override
    public void runOpMode() throws InterruptedException
    {
        armBot = new HardwareArmBot(hardwareMap);

        // put the torso into a known position
        armBot.torsoMotor.setPower(-TORSO_SLOW);
        while (!armBot.torsoLimit.isPressed() && opModeIsActive()) {
            idle();
        }
        armBot.torsoMotor.setPower(TORSO_STOP);

        // reset the encoder
        armBot.torsoMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armBot.torsoMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // bring the arm back to the front


        waitForStart();

        while (opModeIsActive()) {

            controlTorso();
            controlGripper();

            telemetry.update();
            idle();
        }


    }

    private final double GRIPPER_DELTA = 0.1;
    private final double GRIPPER_CLOSED = 0.8;
    private final double GRIPPER_OPEN = 0.2;

    public void controlGripper() {
        double currentPosition = armBot.gripper.getPosition();

        double newPosition = currentPosition;

        // close the gripper
        if (gamepad1.x) newPosition = Range.clip(newPosition + GRIPPER_DELTA,
                GRIPPER_OPEN, GRIPPER_CLOSED);

        // open the gripper
        if (gamepad1.b) newPosition = Range.clip(newPosition - GRIPPER_DELTA,
                GRIPPER_OPEN, GRIPPER_CLOSED);

        /* Enable this once you have good values for GRIPPER_CLOSED and GRIPPER_OPEN
        if (gamepad1.right_bumper) newPosition = GRIPPER_CLOSED;
        if (gamepad1.right_trigger > 0.5) newPosition = GRIPPER_OPEN;
        */

        armBot.gripper.setPosition(newPosition);

        telemetry.addData("Gripper", "position: " + newPosition);
    }


    private final double TORSO_SLOW = 0.1;
    private final double TORSO_NORMAL = 0.2;
    private final double TORSO_STOP = 0.0;
    private final int TORSO_MAX_TURN = 1500;

    public void controlTorso()
    {
        // Torso
        double turn = -gamepad1.left_stick_x * TORSO_NORMAL;

        int torsoEncoder = armBot.torsoMotor.getCurrentPosition();

        // are we hitting the limit switch?
        if (turn < 0.0 && armBot.torsoLimit.isPressed()) {
            turn = TORSO_STOP;
        }

        // are we too far the other way?
        if (turn > 0.0 && torsoEncoder >= TORSO_MAX_TURN) {
            turn = TORSO_STOP;
        }

        // move the torso
        armBot.torsoMotor.setPower(turn);

        // show encoder value to help us set values
        telemetry.addData("TORSO", "Encoder: " + torsoEncoder);
    }
}
