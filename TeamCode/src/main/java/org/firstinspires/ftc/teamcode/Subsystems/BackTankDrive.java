package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BackTankDrive {
    private final Telemetry telemetry;

    public final DcMotor backLeftMotor;
    public final DcMotor backRightMotor;

    public final Gamepad gamepad;

    public BackTankDrive(Telemetry telemetry, DcMotor backLeftMotor, DcMotor backRightMotor, Gamepad gamepad) {
        this.telemetry = telemetry;
        this.backLeftMotor = backLeftMotor;
        this.backRightMotor = backRightMotor;
        this.gamepad = gamepad;
    }

    public void respondToGamepad() {
        backLeftMotor.setPower(gamepad.left_stick_y);
        backRightMotor.setPower(gamepad.right_stick_y);
    }
}
