package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class StrafeDrive {
    private final Telemetry telemetry;

    public final DcMotor frontLeftMotor;
    public final DcMotor frontRightMotor;
    public final DcMotor backLeftMotor;
    public final DcMotor backRightMotor;

    public final Gamepad gamepad;

    public StrafeDrive(Telemetry telemetry, DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor, Gamepad gamepad) {
        this.telemetry = telemetry;

        this.frontLeftMotor = frontLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.backLeftMotor = backLeftMotor;
        this.backRightMotor = backRightMotor;
        this.gamepad = gamepad;
    }

    public void respondToGamepad() {
        float x = gamepad.left_stick_x;
        float y = -gamepad.left_stick_y;
        float turn = gamepad.right_stick_x;

        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double frontLeftPower = power * cos / max + turn;
        double frontRightPower = power * sin / max - turn;
        double backLeftPower = power * sin / max + turn;
        double backRightPower = power * cos / max - turn;

        if (power + Math.abs(turn) > 1) {
            frontLeftPower /= power + turn;
            frontRightPower /= power + turn;
            backLeftPower /= power + turn;
            backRightPower /= power + turn;
        }

        frontLeftPower *= frontLeftPower * frontLeftPower;
        frontRightPower *= frontRightPower * frontRightPower;
        backLeftPower *= backLeftPower * backLeftPower;
        backRightPower *= backRightPower * backRightPower;

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }
}
