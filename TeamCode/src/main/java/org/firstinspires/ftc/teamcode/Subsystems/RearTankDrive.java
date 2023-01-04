package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotOpMode;

public class RearTankDrive implements Subsystem {
    private final RobotOpMode opMode;
    public DcMotor leftMotor, rightMotor;
    private int leftPos = 0;
    private int rightPos = 0;
    private final int turnSensitivity;

    public RearTankDrive(RobotOpMode opMode, int turnSensitivity) {
        this.opMode = opMode;
        this.turnSensitivity = turnSensitivity;
    }

    @Override
    public void setup() {
        leftMotor = opMode.hardwareMap.get(DcMotor.class, "b_left");
        rightMotor = opMode.hardwareMap.get(DcMotor.class, "b_right");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void updateByGamepad() {
        if (opMode.gamepad1.right_stick_x < 0.01 && opMode.gamepad1.right_stick_x > -0.01 && (opMode.gamepad1.left_stick_y > 0.01 || opMode.gamepad1.left_stick_y < -0.01)) {
            // Left-stick control all, going straight
            setPowers(opMode.gamepad1.left_stick_y / 1.5f);
        } else if (opMode.gamepad1.right_stick_x < -0.01 && (opMode.gamepad1.left_stick_y > 0.05 || opMode.gamepad1.left_stick_y < -0.01)) {
            // Turning left while moving
            leftMotor.setPower((opMode.gamepad1.left_stick_y / turnSensitivity) / 2);
            rightMotor.setPower(opMode.gamepad1.left_stick_y / 2);
        } else if (opMode.gamepad1.right_stick_x > 0.01 && (opMode.gamepad1.left_stick_y > 0.05 || opMode.gamepad1.left_stick_y < -0.01)) {
            // Turning right wile moving forward
            leftMotor.setPower(opMode.gamepad1.left_stick_y / 2);
            rightMotor.setPower((opMode.gamepad1.left_stick_y / turnSensitivity) / 2);
        } else if (opMode.gamepad1.right_stick_x < -0.01 && (opMode.gamepad1.left_stick_y > -0.05 || opMode.gamepad1.left_stick_y < 0.01)) {
            // Turning left without going forward
            rightMotor.setPower(opMode.gamepad1.right_stick_x / 2);
            leftMotor.setPower(-(opMode.gamepad1.right_stick_x / 2));
        } else if (opMode.gamepad1.right_stick_x > 0.01 && (opMode.gamepad1.left_stick_y > -0.05 || opMode.gamepad1.left_stick_y < 0.01)) {
            // Turning right without moving forward
            leftMotor.setPower(-(opMode.gamepad1.right_stick_x / 2));
            rightMotor.setPower(opMode.gamepad1.right_stick_x / 2);
        } else if (opMode.gamepad1.right_stick_x == 0 && opMode.gamepad1.left_stick_y == 0) {
            setPowers(0);
        } else {
            setPowers(0);
        }
    }

    public void drive(int leftMove, int rightMove, float speed) {
        leftPos += leftMove;
        rightPos += rightMove;
        leftMotor.setTargetPosition(leftPos);
        rightMotor.setTargetPosition(rightPos);
        setModes(DcMotor.RunMode.RUN_TO_POSITION);
        setPowers(speed);
        opMode.blockOn(leftMotor, rightMotor);
    }

    public void setModes(DcMotor.RunMode mode) {
        leftMotor.setMode(mode);
        rightMotor.setMode(mode);
    }

    public void setPowers(float speed) {
        leftMotor.setPower(speed);
        rightMotor.setPower(speed);
    }
}
