package edu.elon.cs.robotics;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class HardwareArmBot {

    /* MOTORS */
    public DcMotor torsoMotor = null;

    /* SERVOS */
    public Servo gripper = null;

    /* SENSORS */
    public TouchSensor torsoLimit = null;

    /* CONSTANTS */
    public final double GRIPPER_START = 0.5;


    public HardwareArmBot(HardwareMap hardwareMap) {

        /* setup the motors */
        torsoMotor = hardwareMap.get(DcMotor.class, "torsoMotor");
        torsoMotor.setDirection(DcMotor.Direction.FORWARD);

        /* setup the servos */
        gripper = hardwareMap.get(Servo.class, "gripper");
        gripper.setPosition(GRIPPER_START);

        /* setup the sensors */
        torsoLimit = hardwareMap.get(TouchSensor.class, "torsoLimit");
    }

}
