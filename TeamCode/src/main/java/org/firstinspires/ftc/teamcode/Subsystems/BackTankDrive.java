package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BackTankDrive {
    private final Telemetry telemetry;

    public final DcMotor backLeftMotor;
    public final DcMotor backRightMotor;

    public final Gamepad gamepad;

    public final int divideNum;

    public BackTankDrive(Telemetry telemetry, DcMotor backLeftMotor, DcMotor backRightMotor, Gamepad gamepad, int divideNum) {
        this.telemetry = telemetry;
        this.backLeftMotor = backLeftMotor;
        this.backRightMotor = backRightMotor;
        this.gamepad = gamepad;
        this.divideNum = divideNum;
    }

    public void respondToGamepad() {
        if (gamepad.right_stick_x < 0.01 && gamepad.right_stick_x > -0.01 && (gamepad.left_stick_y > 0.01 || gamepad.left_stick_y < -0.01)) {
            // Left-stick control all, going straight
            backLeftMotor.setPower(gamepad.left_stick_y / 1.5);
            backRightMotor.setPower(gamepad.left_stick_y / 1.5);
        } else if (gamepad.right_stick_x < -0.01 && (gamepad.left_stick_y > 0.05 || gamepad.left_stick_y < -0.01)) {
            // Turning left while moving
            backLeftMotor.setPower((gamepad.left_stick_y / divideNum) / 2);
            backRightMotor.setPower(gamepad.left_stick_y / 2);
        } else if (gamepad.right_stick_x > 0.01 && (gamepad.left_stick_y > 0.05 || gamepad.left_stick_y < -0.01)) {
            // Turning right wile moving forward
            backLeftMotor.setPower(gamepad.left_stick_y / 2);
            backRightMotor.setPower((gamepad.left_stick_y / divideNum) / 2);
        } else if (gamepad.right_stick_x < -0.01 && (gamepad.left_stick_y > -0.05 || gamepad.left_stick_y < 0.01)) {
            // Turning left without going forward
            backRightMotor.setPower(gamepad.right_stick_x / 2);
            backLeftMotor.setPower(-(gamepad.right_stick_x / 2));
        } else if (gamepad.right_stick_x > 0.01 && (gamepad.left_stick_y > -0.05 || gamepad.left_stick_y < 0.01)) {
            // Turning right without moving forward
            backLeftMotor.setPower(-(gamepad.right_stick_x / 2));
            backRightMotor.setPower(gamepad.right_stick_x / 2);
        } else if (gamepad.right_stick_x == 0 && gamepad.left_stick_y == 0) {
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
        } else {
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
        }
    }
}
