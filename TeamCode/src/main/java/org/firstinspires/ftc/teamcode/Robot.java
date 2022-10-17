package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
The Robot class is supposed to be a representation of the robot in code.
This class consists of the motor and sensors of the robot.
 */

public class Robot {
    // HardwareMap names
    public static final String FRONT_LEFT_MOTOR_NAME = "motor_front_left";
    public static final String FRONT_RIGHT_MOTOR_NAME = "motor_front_right";
    public static final String BACK_LEFT_MOTOR_NAME = "motor_back_left";
    public static final String BACK_RIGHT_MOTOR_NAME = "motor_back_right";

    // Motors
    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;

    private final Telemetry telemetry;
    private final HardwareMap hardwareMap;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        init();
    }

    private void init() {
        frontLeftMotor = (DcMotor) hardwareMap.get(FRONT_LEFT_MOTOR_NAME);
        backLeftMotor = (DcMotor) hardwareMap.get(BACK_LEFT_MOTOR_NAME);
        frontRightMotor = (DcMotor) hardwareMap.get(FRONT_RIGHT_MOTOR_NAME);
        backRightMotor = (DcMotor) hardwareMap.get(BACK_RIGHT_MOTOR_NAME);

        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetryBroadcast("Status", "Initialized hardware map");
    }

    public void telemetryBroadcast(String caption, String value) {
        telemetry.addData(caption, value);
        telemetry.update();
    }
}
